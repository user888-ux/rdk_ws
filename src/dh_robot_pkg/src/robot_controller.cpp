#include "dh_robot_pkg/robot_controller.hpp"
#include "dh_robot_pkg/serial_communicator.hpp"
#include "dh_robot_pkg/dh_kinematics.hpp"
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "dh_robot_pkg/robot_controller.hpp"

//接收->放入缓冲区->判断数据是否有效->是：解算->发布
//                             ->否：不断丢弃头帧

RobotController::RobotController()
    : Node("robot_controller") {
    
    // 创建串口通信器
    serial_ = std::make_unique<SerialCommunicator>(this);
    
    // 设置二进制数据回调
    serial_->setBinaryCallback(
        [this](const uint8_t* data, size_t len) {
            this->onSerialData(data, len);
        });
    
    // 连接串口（请根据实际设备名和波特率修改）
    const std::string port = "/dev/pts/5";
    const unsigned int baud = 115200;
    if (!serial_->connect(port, baud)) {
        RCLCPP_ERROR(get_logger(), "Failed to open serial port %s", port.c_str());
        rclcpp::shutdown();
    }
}

void RobotController::onSerialData(const uint8_t* data, size_t len) {
    // 追加数据到累积缓冲区。insert对于多字节比push_back()更高效
    rx_buffer_.insert(rx_buffer_.end(), data, data + len);
    
    // 尝试解析完整帧
    parseFrames();
}

void RobotController::parseFrames() {
    // 帧尾特征序列
    static const uint8_t TAIL_SEQ[4] = {0x00, 0x00, 0x80, 0x7f};
    
    while (rx_buffer_.size() >= 4) {
        // 在缓冲区中搜索帧尾
        auto tail_pos = std::search(rx_buffer_.begin(), rx_buffer_.end(),
                                    TAIL_SEQ, TAIL_SEQ + 4);
        
        if (tail_pos == rx_buffer_.end()) {
            // 没找到帧尾，保留最后3个字节（防止帧尾跨包）
            size_t keep = std::min(rx_buffer_.size(), size_t(3));
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.end() - keep);
            break;
        }
        
        // 帧尾位置到开头的距离 = 浮点数据总字节数
        size_t data_bytes = std::distance(rx_buffer_.begin(), tail_pos);
        
        // 必须是4的倍数（每个float占4字节）
        if (data_bytes % 4 == 0) {
            // 有效帧
            size_t float_count = data_bytes / 4;
            std::vector<float> values(float_count);
            std::memcpy(values.data(), rx_buffer_.data(), data_bytes);
            
            // 处理这一帧的数据
            processFrame(values);
            
            // 删除已处理的数据 + 帧尾
            rx_buffer_.erase(rx_buffer_.begin(), tail_pos + 4);
        } else {
            // 字节数不对，丢弃1字节重新搜索
            rx_buffer_.erase(rx_buffer_.begin());
        }
    }
}

void RobotController::processFrame(const std::vector<float>& values) {
    if (values.size() < 2) {
        RCLCPP_WARN(get_logger(), "Frame too short, expected at least 2 values");
        return;
    }
    
    float pitch = values[0];
    float yaw   = values[1];
    RCLCPP_INFO(get_logger(), "Successfully receive data:pitch=%f,yaw=%f",pitch,yaw);

    
    // 你原来的解算逻辑保持不变
    // ...
    float new_pitch=0;
    float new_yaw=0;
    
    // 发送响应时，如果通道数不变，继续发2个浮点数即可
    sendResponse(new_pitch, new_yaw);
}

void RobotController::sendResponse(float new_pitch, float new_yaw) {
    std::vector<uint8_t> combined_frame;

    // 1. 添加 yaw 帧头、标识、长度
    combined_frame.push_back(0xAA);
    combined_frame.push_back(0x01);
    combined_frame.push_back(0x04);

    // 添加 yaw 浮点数 4 字节（保持原内存顺序）
    uint8_t* yaw_bytes = reinterpret_cast<uint8_t*>(&new_yaw);
    combined_frame.insert(combined_frame.end(), yaw_bytes, yaw_bytes + 4);

    // 添加 yaw 帧尾
    combined_frame.push_back(0x0D);

    // 2. 添加 pitch 帧头、标识、长度
    combined_frame.push_back(0xAA);
    combined_frame.push_back(0x02);
    combined_frame.push_back(0x04);

    // 添加 pitch 浮点数 4 字节
    uint8_t* pitch_bytes = reinterpret_cast<uint8_t*>(&new_pitch);
    combined_frame.insert(combined_frame.end(), pitch_bytes, pitch_bytes + 4);

    // 添加 pitch 帧尾
    combined_frame.push_back(0x0D);

    // 一次性发送合并后的完整数据包
    if (!serial_->send(combined_frame)) {
        RCLCPP_WARN(get_logger(), "Failed to send combined yaw/pitch response frame");
    }
}