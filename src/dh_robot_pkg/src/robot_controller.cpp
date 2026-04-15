#include "dh_robot_pkg/robot_controller.hpp"
#include "dh_robot_pkg/serial_communicator.hpp"
#include "dh_robot_pkg/dh_kinematics.hpp"
#include <memory>
#include "rclcpp/rclcpp.hpp"

//这代码干了5件事
// 订阅关节状态信息
// 计算机器人正向运动学
// 发布末端执行器位姿,TF位姿并在RViz显示
// 完成与串口的通信
// 接收修改的参数并完成二次计算和发布

namespace dh_robot_pkg {

RobotController::RobotController() 
    //所有在 .hpp定义的都要初始化（血泪教训，否则它能编译但会出 bug）
    : Node("robot_controller"),serial_communicator_(this),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)),
    robot_model_ (std::make_shared<RobotModel>())
    {
    
    // 参数声明
    this->declare_parameter<std::string>("robot_type", "scara");
    this->declare_parameter<bool>("publish_tf", false);
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("serial_baudrate", 115200);
    // 方便实时在终端更新参数
    this->declare_parameter<double>("joint1_position", 0.0);
    this->declare_parameter<double>("joint2_position", 0.0);
    this->declare_parameter<double>("joint3_position", 0.0);

    // 添加参数回调
    param_sub_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
            return this->parametersCallback(params);
        });
    
    // 获取参数
    robot_type_ = this->get_parameter("robot_type").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    std::string serial_port = this->get_parameter("serial_port").as_string();
    int baudrate = this->get_parameter("serial_baudrate").as_int();

    //参数可以在启动时动态设置：
    //ros2 run dh_robot_pkg robot_controller --ros-args -p robot_type:=articulated
    
    // 根据类型设置机器人模型
    if (robot_type_ == "scara") {
        robot_model_->setupSCARAModel();
        RCLCPP_INFO(get_logger(), "Using SCARA robot model");
    } else if (robot_type_ == "articulated") {
        robot_model_->setupArticulatedModel();
        RCLCPP_INFO(get_logger(), "Using Articulated robot model");
    } else {
        robot_model_->setupSCARAModel();
        RCLCPP_INFO(get_logger(), "Using default SCARA robot model");
    }
    
    // 建立与串口的通信
    setupSerialCommunication(serial_port, baudrate);

    // 通信设置：
    // 创建订阅者
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,//订阅话题 "joint_states"(关节状态),队列大小为 10
        std::bind(&RobotController::jointStateCallback, this, std::placeholders::_1));
    
    // 创建发布者
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "end_effector_pose", 10);//发布话题 "end_effector_pose"(末端位姿),队列大小为 10
    
    RCLCPP_INFO(get_logger(), "Robot controller initialized successfully");
}

//建立与串口的通信
void RobotController::setupSerialCommunication(const std::string& port, int baudrate) {
    // 设置串口接收回调
    serial_communicator_.setReceiveCallback(
        [this](const std::string& data) {
            this->handleSerialData(data);
        }
    );
    
    // 连接串口
    if (serial_communicator_.connect(port, baudrate)) {
        RCLCPP_INFO(get_logger(), "Serial communication established");
    } else {
        RCLCPP_WARN(get_logger(), "Failed to establish serial communication");
    }
}

void RobotController::handleSerialData(const std::string& data) {
    try {
        // 解析从STM32接收到的关节数据
        // 假设数据格式为: "joint1,joint2,joint3"
        std::vector<double> joint_positions;
        std::stringstream ss(data);
        std::string token;
        
        while (std::getline(ss, token, ',')) {
            joint_positions.push_back(std::stod(token));
        }
        
        if (joint_positions.size() >= 3) {
            // 创建JointState消息并发布
            auto joint_msg = std::make_unique<sensor_msgs::msg::JointState>();
            joint_msg->header.stamp = this->now();
            joint_msg->header.frame_id = "base_link";
            joint_msg->name = {"joint1", "joint2", "joint3"};
            joint_msg->position = joint_positions;
            joint_msg->velocity = {0.0, 0.0, 0.0};
            joint_msg->effort = {0.0, 0.0, 0.0};
            
            // 调用关节状态回调函数来处理
            jointStateCallback(std::move(joint_msg));
            
            RCLCPP_DEBUG(get_logger(), "Processed joint data from serial: %s", data.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Failed to parse serial data: %s", data.c_str());
    }
}

//jointStateCallback和 parametersCallback都会用到该函数
void RobotController::updateRobotPoseAndTF(const std::vector<double>& joint_angles, const builtin_interfaces::msg::Time& stamp) {
    try {
        auto kinematics = robot_model_->getKinematics();
        if (!kinematics) {
            RCLCPP_ERROR(get_logger(), "Kinematics not available");
            return;
        }
        
        // 1. 计算正向运动学
        KDL::Frame end_effector = kinematics->forwardKinematics(joint_angles);
        
        // 2. 发布末端执行器位姿
        auto pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
        pose_msg->header.stamp = stamp;
        pose_msg->header.frame_id = "base_link";
        
        pose_msg->pose.position.x = end_effector.p.x();
        pose_msg->pose.position.y = end_effector.p.y();
        pose_msg->pose.position.z = end_effector.p.z();

        double x, y, z, w;
        end_effector.M.GetQuaternion(x, y, z, w);
        pose_msg->pose.orientation.x = x;
        pose_msg->pose.orientation.y = y;
        pose_msg->pose.orientation.z = z;
        pose_msg->pose.orientation.w = w;
        pose_pub_->publish(std::move(pose_msg));

        // 3. 发布所有关节的TF变换
        publishAllJointTransforms(joint_angles, stamp);

        RCLCPP_DEBUG(get_logger(), "Updated robot pose and TF from joint angles");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in updateRobotPoseAndTF: %s", e.what());
    }
}

//订阅者的回调函数
void RobotController::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    try {
        if (msg->position.empty()) {
            RCLCPP_WARN(get_logger(), "Received empty joint state message");
            return;
        }

        // 直接调用公共函数，传入消息中的关节角度和时间戳
        updateRobotPoseAndTF(msg->position, msg->header.stamp);

        // 将计算得到的位姿发送回STM32(暂未启用)
        // std::stringstream pose_data;
        // pose_data << std::fixed << std::setprecision(6);
        // pose_data << end_effector.p.x() << "," 
        //          << end_effector.p.y() << "," 
        //          << end_effector.p.z() << ","
        //          << x << "," << y << "," << z << "," << w << "\n";
        
        // serial_communicator_.send(pose_data.str());
        
        RCLCPP_DEBUG(get_logger(), "Published end effector pose");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in joint state callback: %s", e.what());
    }
}

// 设置参数的回调函数
rcl_interfaces::msg::SetParametersResult RobotController::parametersCallback(
    const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "参数更新成功";

    for (const auto& param : params) {
        if (param.get_name() == "joint1_position") joint1_pos_ = param.as_double();
        else if (param.get_name() == "joint2_position") joint2_pos_ = param.as_double();
        else if (param.get_name() == "joint3_position") joint3_pos_ = param.as_double();
        else if (param.get_name() == "publish_tf") publish_tf_ = param.as_bool();
    }

    // 调用公共函数，传入成员变量的关节角度和当前节点时间
    std::vector<double> joint_angles = {joint1_pos_, joint2_pos_, joint3_pos_};
    updateRobotPoseAndTF(joint_angles, this->now());

    return result;
}

//发布 TF 数据给 RViz 可视化
void RobotController::publishAllJointTransforms(const std::vector<double>& joint_angles, 
                                               const builtin_interfaces::msg::Time& stamp) {
    //初始化数学库的计算器，通过它来调用数学库里的函数
    auto kinematics = robot_model_->getKinematics();
    if (!kinematics) return;
    
    // 对于SCARA机器人，我们有3个关节+1个末端执行器
    KDL::Frame current_pose = KDL::Frame::Identity();
    
    // 获取D-H参数
    auto dh_params = robot_model_->getDHParameters();
    
    for (size_t i = 0; i < dh_params.size(); ++i) {
        double joint_value = (i < joint_angles.size()) ? joint_angles[i] : 0.0;
        
        // 计算当前连杆的变换
        KDL::Frame transform = kinematics->calculateDHTransform(dh_params[i], joint_value);
        current_pose = current_pose * transform;
        
        // 发布当前关节的TF
        std::string parent_frame, child_frame;
        
        if (i == 0) {
            parent_frame = "base_link";
            child_frame = "joint1_link";
        } else if (i == 1) {
            parent_frame = "joint1_link";
            child_frame = "joint2_link";
        } else if (i == 2) {
            parent_frame = "joint2_link";
            child_frame = "joint3_link";
        } else if (i == 3) {
            parent_frame = "joint3_link";
            child_frame = "end_effector_link";
        }
        
        publishSingleTFTransform(current_pose, stamp, parent_frame, child_frame);
    }
}

// 发布单个TF变换的辅助函数
void RobotController::publishSingleTFTransform(const KDL::Frame& transform, 
                                              const builtin_interfaces::msg::Time& stamp,
                                              const std::string& parent_frame,
                                              const std::string& child_frame) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    
    transform_stamped.header.stamp = stamp;
    transform_stamped.header.frame_id = parent_frame;
    transform_stamped.child_frame_id = child_frame;
    
    transform_stamped.transform.translation.x = transform.p.x();
    transform_stamped.transform.translation.y = transform.p.y();
    transform_stamped.transform.translation.z = transform.p.z();
    
    double x, y, z, w;
    transform.M.GetQuaternion(x, y, z, w);
    transform_stamped.transform.rotation.x = x;
    transform_stamped.transform.rotation.y = y;
    transform_stamped.transform.rotation.z = z;
    transform_stamped.transform.rotation.w = w;
    
    tf_broadcaster_->sendTransform(transform_stamped);
}

} // namespace dh_robot_pkg

// 移除组件注册，使用标准main函数