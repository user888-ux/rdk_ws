#include "dh_robot_pkg/robot_controller.hpp"
#include "dh_robot_pkg/serial_communicator.hpp"
// #include "dh_robot_pkg/dh_kinematics.hpp"
#include "dh_robot_pkg/stereo_system.hpp"
#include <memory>
#include "rclcpp/rclcpp.hpp"

//接收->放入缓冲区->判断数据是否有效->是：解算->发布
//                             ->否：不断丢弃头帧

RobotController::RobotController()
    : Node("robot_controller"),
      start_time_(std::chrono::steady_clock::now())
{

    // 创建串口通信器
    serial_ = std::make_unique<SerialCommunicator>(this);
    
    // 业务逻辑不再需要接收数据
    // 设置二进制数据回调
    // serial_->setBinaryCallback(
    //     [this](const uint8_t* data, size_t len) {
    //         this->onSerialData(data, len);
    //     });
    
    // 连接串口（根据命令行获取实际设备名和波特率修改）
    this->declare_parameter<std::string>("port", "/dev/ttyS1");
    this->declare_parameter<int>("baud", 500000);
    this->declare_parameter<bool>("mode", false); //默认回调而不是定时器
    this->declare_parameter<double>("frame_rate", 30.0); 
    this->declare_parameter<double>("max_rel_err", 0.25);
    this->declare_parameter<int>("max_buffer_size", 10);
    this->declare_parameter<std::string>("yaml_path", "config/stereo_calib.yaml");
    std::string port = this->get_parameter("port").as_string();
    int baud = this->get_parameter("baud").as_int();
    mode= this->get_parameter("mode").as_bool();
    double frame_rate = this->get_parameter("frame_rate").as_double();
    double max_rel_err = this->get_parameter("max_rel_err").as_double();
    int max_buffer_size = this->get_parameter("max_buffer_size").as_int();
    std::string yaml_path = this->get_parameter("yaml_path").as_string();

    if (!serial_->connect(port, baud)) {
        RCLCPP_ERROR(get_logger(), "Failed to open serial port %s", port.c_str());
        rclcpp::shutdown();
    }

    if (frame_rate <= 0.0) {
        RCLCPP_ERROR(get_logger(), "Invalid frame_rate: %f, must be >0", frame_rate);
        rclcpp::shutdown();
    }

    if(getMode()==true){//定时器模式
        timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / frame_rate),
        [this]() { this->timer_callback(); }
        );
        // AI推理消息的订阅者
        ai_subscription_ = this->create_subscription<PerceptionTargets>(
            "hobot_dnn_detection",
            10,
            // 但实际上我们不使用回调，所以可以传一个空的lambda，但是必须存在
            [](const PerceptionTargets::SharedPtr) {}
        );
    }else{//回调模式
        ai_subscription_ = this->create_subscription<PerceptionTargets>(
            "hobot_dnn_detection",
            10,
            [this](const PerceptionTargets::SharedPtr msg) {
                this->ai_subscription_callback(msg);
            }
        );
    }

    stereo_ = std::make_unique<StereoSystem>(yaml_path);
    stereo_->setFilterParams(max_rel_err, max_buffer_size); // 设置滤波参数
}

void RobotController::timer_callback()
{
    // 创建一个消息对象用于接收数据
    PerceptionTargets msg;
    rclcpp::MessageInfo msg_info;   // 包含时间戳、发布者信息等（可选）

    float target_u=0.0;float target_v=0.0; // 准备发送出去的数据
    float target_dist=1.0;float find_target=0.0;
    if (ai_subscription_->take(msg,msg_info)) {
        // 遍历目标
        if(msg.targets.size()==1){ // 只有一个目标
            const auto& target = msg.targets[0]; 
            const auto& roi = target.rois[0];
            const auto& rect = roi.rect;
            target_u = rect.x_offset + rect.width / 2.0f;
            target_v = rect.y_offset + rect.height / 2.0f;
            find_target=1.0;
            if(0<target_u && target_u<=640){//目标只在左目。这里640暂时硬编码，以后从yaml中获取
                sendResponse(target_u,target_v,target_dist,find_target);
                RCLCPP_INFO(this->get_logger(),
                    "只检测到左目有目标 | 类型: %s, ID: %lu, 坐标: (%.1f, %.1f), 置信度: %.3f, 标志位: %.1f",
                    target.type.c_str(),
                    target.track_id,
                    target_u, target_v,
                    roi.confidence,
                    find_target);
            }else if(640<target_u && target_u<=1280){//目标只在右目,根据业务逻辑将target_u强制设为640
                sendResponse(640,target_v,target_dist,find_target);
                RCLCPP_INFO(this->get_logger(),
                    "只检测到右目有目标,将target_u强制设为640 | 类型: %s, ID: %lu, 坐标: (%.1f, %.1f), 置信度: %.3f, 标志位: %.1f",
                    target.type.c_str(),
                    target.track_id,
                    target_u, target_v,
                    roi.confidence,
                    find_target);
            }else{
                //超出范围警告
                find_target=0.0f;
                sendResponse(0.0,0.0,1.0,find_target);
                RCLCPP_WARN(this->get_logger(), "目标坐标(%.1f,%.1f)超出合理范围", target_u, target_v);
            }
        }else if (msg.targets.size() == 2){
            // 提取两个目标的原始数据
            const auto& target1 = msg.targets[0];
            const auto& target2 = msg.targets[1];

            // 每个target只有一个roi，索引为0
            const auto& roi1 = target1.rois[0];
            const auto& roi2 = target2.rois[0];

            const auto& rect1 = roi1.rect;
            const auto& rect2 = roi2.rect;

            float u1 = rect1.x_offset + rect1.width / 2.0f;
            float v1 = rect1.y_offset + rect1.height / 2.0f;
            float u2 = rect2.x_offset + rect2.width / 2.0f;
            float v2 = rect2.y_offset + rect2.height / 2.0f;

            // 区分左右目标（根据x坐标）
            float u_left, v_left, u_right, v_right;
            if (u1 < u2) {
                u_left = u1; v_left = v1;
                u_right = u2; v_right = v2;
            } else {
                u_left = u2; v_left = v2;
                u_right = u1; v_right = v1;
            }

            // 检查左右目标的x坐标是否在有效范围内 [0, 1280]
            const float IMAGE_WIDTH = 1280.0f;
            bool left_valid = (u_left >= 0.0f && u_left <= IMAGE_WIDTH);
            bool right_valid = (u_right >= 0.0f && u_right <= IMAGE_WIDTH);

            if (left_valid && right_valid) {
                // 左右各一目标，且都在范围内
                find_target = 1.0f;

                //测距
                double dist = this->stereo_->computeDistance(u1, v1, u2, v2);
                if (0<dist && dist<=10.0) {
                    sendResponse(u_left,v_left,dist,find_target);// 根据业务要求只需要发左目uv
                    RCLCPP_INFO(this->get_logger(),
                        "双目标 | 左目: (%.1f, %.1f) 置信度: %.3f, 右目: (%.1f, %.1f) 置信度: %.3f, 标志位: %.1f, 距离:%.3f",
                        u_left, v_left, roi1.confidence,
                        u_right, v_right, roi2.confidence,
                        find_target,dist);
                } else{
                    find_target=0.0f;
                    sendResponse(0.0,0.0,1.0,find_target);
                    RCLCPP_WARN(this->get_logger(), 
                    "距离 %.3f 不合理", dist);
                }
            } else {
                // 坐标超出范围，标记无效
                find_target = 0.0f;
                sendResponse(0.0,0.0,1.0,find_target);
                RCLCPP_WARN(this->get_logger(), 
                    "目标超出图像边界: left_valid=%d, right_valid=%d", 
                    left_valid, right_valid);
            }
        }else if(msg.targets.size()>=3){
            find_target=0.0f;
            sendResponse(0.0,0.0,1.0,find_target);
            RCLCPP_WARN(this->get_logger(),"三个以上目标无法判断");
        }

    } else { 
        find_target=0.0f;
        sendResponse(0.0,0.0,1.0,find_target);
        RCLCPP_WARN(this->get_logger(), "没有检测到目标");
    }
}

void RobotController::ai_subscription_callback(const PerceptionTargets::SharedPtr msg)
{
    // 直接使用 msg，不再需要 take()
    float target_u = 0.0, target_v = 0.0;
    float target_dist = 1.0, find_target = 0.0;

    // 注意：msg 永远不会为空指针，但 targets 可能为空
    if (msg->targets.empty()) {
        // 没有目标
        find_target=0.0f;
        sendResponse(0.0,0.0,1.0,find_target);
        RCLCPP_DEBUG(this->get_logger(), "没有检测到目标");
        return;
    }

    // 处理单个目标
    if (msg->targets.size() == 1) {
        const auto& target = msg->targets[0];
        const auto& roi = target.rois[0];
        const auto& rect = roi.rect;
        target_u = rect.x_offset + rect.width / 2.0f;
        target_v = rect.y_offset + rect.height / 2.0f;
        find_target = 1.0;

        if (0 < target_u && target_u <= 640) {
            // 左目
            sendResponse(target_u, target_v, target_dist, find_target);
            RCLCPP_INFO(this->get_logger(),
                "只检测到左目有目标 | 类型: %s, ID: %lu, 坐标: (%.1f, %.1f), 置信度: %.3f, 标志位: %.1f",
                target.type.c_str(), target.track_id,
                target_u, target_v, roi.confidence, find_target);
        } else if (640 < target_u && target_u <= 1280) {
            // 右目强制设为 640
            find_target=0;//新的业务逻辑也不需要右目了
            sendResponse(640, target_v, target_dist, find_target);
            RCLCPP_INFO(this->get_logger(),
                "只检测到右目有目标,将target_u强制设为640 | 类型: %s, ID: %lu, 坐标: (%.1f, %.1f), 置信度: %.3f, 标志位: %.1f",
                target.type.c_str(), target.track_id,
                target_u, target_v, roi.confidence, find_target);
        } else {
            RCLCPP_WARN(this->get_logger(), "目标坐标(%.1f,%.1f)超出合理范围", target_u, target_v);
        }
    }
    // 处理两个目标
    else if (msg->targets.size() == 2) {
        const auto& target1 = msg->targets[0];
        const auto& target2 = msg->targets[1];
        const auto& roi1 = target1.rois[0];
        const auto& roi2 = target2.rois[0];
        const auto& rect1 = roi1.rect;
        const auto& rect2 = roi2.rect;

        float u1 = rect1.x_offset + rect1.width / 2.0f;
        float v1 = rect1.y_offset + rect1.height / 2.0f;
        float u2 = rect2.x_offset + rect2.width / 2.0f;
        float v2 = rect2.y_offset + rect2.height / 2.0f;

        float u_left, v_left, u_right, v_right;
        if (u1 < u2) {
            u_left = u1; v_left = v1;
            u_right = u2; v_right = v2;
        } else {
            u_left = u2; v_left = v2;
            u_right = u1; v_right = v1;
        }

        const float IMAGE_WIDTH = 1280.0f;
        bool left_valid = (u_left >= 0.0f && u_left <= IMAGE_WIDTH);
        bool right_valid = (u_right >= 0.0f && u_right <= IMAGE_WIDTH);

        if (left_valid && right_valid) {
            find_target = 1.0f;

            //测距
            double dist = this->stereo_->computeDistance(u1, v1, u2, v2);
            if (0<dist && dist<=10.0) {
                sendResponse(u_left,v_left,dist,find_target);// 根据业务要求只需要发左目uv
                RCLCPP_INFO(this->get_logger(),
                    "双目标 | 左目: (%.1f, %.1f) 置信度: %.3f, 右目: (%.1f, %.1f) 置信度: %.3f, 标志位: %.1f, 距离:%.3f",
                    u_left, v_left, roi1.confidence,
                    u_right, v_right, roi2.confidence,
                    find_target,dist);
            } else{
                RCLCPP_WARN(this->get_logger(), 
                "距离 %.3f 不合理", dist);
                find_target=0.0f;
                sendResponse(0.0,0.0,1.0,find_target);//找不到时距离强制设置为1.0f,目标坐标设置为0
            }

        } else {
            find_target = 0.0f;
            RCLCPP_WARN(this->get_logger(),
                "目标超出图像边界: left_valid=%d, right_valid=%d",
                left_valid, right_valid);
            sendResponse(0.0,0.0,1.0,find_target);
        }
    }
    // 三个及以上目标
    else {
        find_target=0.0f;
        sendResponse(0.0,0.0,1.0,find_target);
        RCLCPP_WARN(this->get_logger(), "三个以上目标无法判断");
    }
}

// 后来业务逻辑不需要我再接收信息，已弃用
// void RobotController::onSerialData(const uint8_t* data, size_t len) {
//     // 追加数据到累积缓冲区。insert对于多字节比push_back()更高效
//     rx_buffer_.insert(rx_buffer_.end(), data, data + len);
//     LOG("已经将数据加到缓冲区"<<rx_buffer_[0]<<' '<<rx_buffer_[1]<<' '<<rx_buffer_[2]<<' '<<rx_buffer_[3]<<' ');
    
//     // 尝试解析完整帧
//     parseFrames();
// }

// 后来业务逻辑不需要我再接收信息，已弃用
// void RobotController::parseFrames() {
//     // 帧尾特征序列
//     static const uint8_t TAIL_SEQ[4] = {0x00, 0x00, 0x80, 0x7f};

//     // 打印缓冲区内容，用于调试
//     std::stringstream ss;
//     ss << "Buffer (" << rx_buffer_.size() << " bytes): ";
//     for (uint8_t c : rx_buffer_) {
//         ss << std::hex << std::setw(2) << std::setfill('0') << (int)c << " ";
//     }
//     LOG(ss.str());
    
//     while (rx_buffer_.size() >= 4) {
//         // 在缓冲区中搜索帧尾
//         auto tail_pos = std::search(rx_buffer_.begin(), rx_buffer_.end(),
//                                     TAIL_SEQ, TAIL_SEQ + 4);
        
//         if (tail_pos == rx_buffer_.end()) {
//             // 没找到帧尾，但缓冲区可能含有帧尾的一部分
//             // 如果缓冲区太大（比如超过最大帧长+4），说明数据错误，清空缓冲区
//             const size_t MAX_BUFFER = 1024;  // 根据你的最大帧长设定
//             if (rx_buffer_.size() > MAX_BUFFER) {
//                 LOG("缓冲区溢出，清空");
//                 rx_buffer_.clear();
//             }
//             // 否则什么都不做，等待更多数据
//             break;
//         }
        
//         // 帧尾位置到开头的距离 = 浮点数据总字节数
//         size_t data_bytes = std::distance(rx_buffer_.begin(), tail_pos);
        
//         // 必须是4的倍数（每个float占4字节）
//         if (data_bytes % 4 == 0) {
//             // 有效帧
//             size_t float_count = data_bytes / 4;
//             std::vector<float> values(float_count);
//             std::memcpy(values.data(), rx_buffer_.data(), data_bytes);
            
//             // 处理这一帧的数据
//             LOG("处理这一帧的数据");
//             processFrame(values);
            
//             // 删除已处理的数据 + 帧尾
//             rx_buffer_.erase(rx_buffer_.begin(), tail_pos + 4);
//         } else {
//             // 字节数不对，丢弃1字节重新搜索
//             rx_buffer_.erase(rx_buffer_.begin());
//         }
//     }
// }

// 后来业务逻辑不需要我再接收信息，已弃用
// void RobotController::processFrame(const std::vector<float>& values) {
//     if (values.size() < 2) {
//         RCLCPP_WARN(this->get_logger(), "Frame too short, expected at least 2 values");
//         return;
//     }
    
//     float pitch = values[values.size()-1];
//     float yaw   = values[values.size()-2];
//     if(pitch<-180 || pitch>180 || yaw <-180 || yaw>180){LOG("数据超出范围，排除");return;}
//     RCLCPP_INFO(get_logger(), "Successfully receive data:pitch=%f,yaw=%f",pitch,yaw);

    
//     // 你原来的解算逻辑保持不变
//     // ...
//     float new_pitch=pitch+0.1;
//     float new_yaw=yaw+0.1;
    
//     // 发送响应时，如果通道数不变，继续发2个浮点数即可
//     sendResponse(new_pitch, new_yaw);
// }

// 不需要再发送以时间为变量的角度，已弃用
// void RobotController::sendResponse()
// {
//     // 获取当前时间相对于启动时刻的秒数 t
//     auto now = std::chrono::steady_clock::now();
//     double t = std::chrono::duration<double>(now - start_time_).count();

//     // 限制 t 在 [0, 8) 范围内循环
//     double t_cycle = std::fmod(t, 8.0);

//     // 计算角度
//     double yaw   = 45.0 * t_cycle - 180.0;                         // yaw = 45t - 180
//     double pitch = 30.0 * sin(15.0 * M_PI / 180.0 * t_cycle);     // pitch = 30·sin(15·π/180·t)

//     // 可选：降低日志输出频率，以免控制台刷屏（例如每 1 秒打印一次）
//     static int64_t last_print = 0;
//     int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
//     if (now_ms - last_print > 1000) {   // 每秒打印一次
//         RCLCPP_INFO(get_logger(), "Send: t_cycle=%.3f, pitch=%.3f, yaw=%.3f", t_cycle, pitch, yaw);
//         last_print = now_ms;
//     }

//     // 构建发送帧（与原来完全相同）
//     std::vector<uint8_t> combined_frame;

//     // yaw 帧 (ID 0x01)
//     combined_frame.push_back(0xAA);
//     combined_frame.push_back(0x18);
//     combined_frame.push_back(0x04);
//     uint8_t* yaw_bytes = reinterpret_cast<uint8_t*>(&yaw);
//     combined_frame.insert(combined_frame.end(), yaw_bytes, yaw_bytes + 4);
//     combined_frame.push_back(0x0D);

//     // pitch 帧 (ID 0x02)
//     combined_frame.push_back(0xAA);
//     combined_frame.push_back(0x19);
//     combined_frame.push_back(0x04);
//     uint8_t* pitch_bytes = reinterpret_cast<uint8_t*>(&pitch);
//     combined_frame.insert(combined_frame.end(), pitch_bytes, pitch_bytes + 4);
//     combined_frame.push_back(0x0D);

//     // 发送
//     if (!serial_->send(combined_frame)) {
//         RCLCPP_WARN(get_logger(), "Failed to send time-based response frame");
//     }
// }

void RobotController::sendResponse(float target_lu,float target_lv,float target_dist,float find_target)
{
    // 辅助函数：将 float 转换为小端序的 4 字节数组
    auto floatToLeBytes = [](float value, uint8_t out[4]) {
        uint8_t raw[4];
        memcpy(raw, &value, 4);  // 获取 float 的内存表示

        // 判断系统字节序
        static const bool is_little_endian = []() {
            uint32_t test = 1;
            return *reinterpret_cast<uint8_t*>(&test) == 1;
        }();

        if (is_little_endian) {
            // 小端系统：直接复制
            out[0] = raw[0];
            out[1] = raw[1];
            out[2] = raw[2];
            out[3] = raw[3];
        } else {
            // 大端系统：反转顺序，输出小端序
            out[0] = raw[3];
            out[1] = raw[2];
            out[2] = raw[1];
            out[3] = raw[0];
        }
    };

    // 构建发送帧（逐字节构建，方便调试）
    std::vector<uint8_t> combined_frame;

    // target_lu 帧 (ID 0x20)
    combined_frame.push_back(0xAA);
    combined_frame.push_back(0x20);
    combined_frame.push_back(0x04);
    uint8_t target_lu_le[4];
    floatToLeBytes(target_lu, target_lu_le);
    combined_frame.insert(combined_frame.end(), target_lu_le, target_lu_le + 4);
    combined_frame.push_back(0x0D);

    // target_lv 帧 (ID 0x21)
    combined_frame.push_back(0xAA);
    combined_frame.push_back(0x21);
    combined_frame.push_back(0x04);
    uint8_t target_lv_le[4];
    floatToLeBytes(target_lv, target_lv_le);
    combined_frame.insert(combined_frame.end(), target_lv_le, target_lv_le + 4);
    combined_frame.push_back(0x0D);

    // target_dist 帧(ID 0x22)
    combined_frame.push_back(0xAA);
    combined_frame.push_back(0x22);
    combined_frame.push_back(0x04);
    uint8_t target_dist_le[4];
    floatToLeBytes(target_dist, target_dist_le);
    combined_frame.insert(combined_frame.end(), target_dist_le, target_dist_le + 4);
    combined_frame.push_back(0x0D);

    // find_target 帧(ID 0x23)
    combined_frame.push_back(0xAA);
    combined_frame.push_back(0x23);
    combined_frame.push_back(0x04);
    uint8_t find_target_le[4];
    floatToLeBytes(find_target, find_target_le);
    combined_frame.insert(combined_frame.end(), find_target_le, find_target_le + 4);
    combined_frame.push_back(0x0D);

    // 逐个字节发送（调试用）
    for (size_t i = 0; i < combined_frame.size(); ++i) {
        std::vector<uint8_t> single_byte = { combined_frame[i] };
        if (!serial_->send(single_byte)) {
            RCLCPP_WARN(this->get_logger(), "Failed to send byte %zu (0x%02X)", i, combined_frame[i]);
            break;
        }
    }
}
