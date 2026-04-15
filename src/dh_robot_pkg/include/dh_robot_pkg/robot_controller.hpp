#pragma once

//第三方库
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <kdl/frames.hpp>
#include <memory>
#include <string>
#include <vector>
//自己写的库
#include "dh_robot_pkg/robot_model.hpp"
#include "dh_robot_pkg/serial_communicator.hpp"
#include "dh_robot_pkg/dh_kinematics.hpp"

namespace dh_robot_pkg {

class RobotController : public rclcpp::Node {
public:
    RobotController();  // 移除NodeOptions参数
    
private:
    //通信回调
    void updateRobotPoseAndTF(const std::vector<double>& joint_angles, const builtin_interfaces::msg::Time& stamp);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter>& params);
    //TF函数及其辅助函数
    void publishAllJointTransforms(const std::vector<double>& joint_angles, const builtin_interfaces::msg::Time& stamp);
    void publishSingleTFTransform(const KDL::Frame& transform, 
                                              const builtin_interfaces::msg::Time& stamp,
                                              const std::string& parent_frame,
                                              const std::string& child_frame) ;
    // 串口通信相关方法
    void setupSerialCommunication(const std::string& port, int baudrate);
    void handleSerialData(const std::string& data);

    // 串口通信器
    SerialCommunicator serial_communicator_;

    // TF广播器
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  
        
    // 运动学模型
    std::shared_ptr<RobotModel> robot_model_;

    //自己在终端手动设置参数时的设置回调函数的设置器
    OnSetParametersCallbackHandle::SharedPtr param_sub_;
    
    // ROS2相关
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    // 参数
    std::string robot_type_;
    bool publish_tf_;

    double joint1_pos_;
    double joint2_pos_;
    double joint3_pos_;
};

} // namespace dh_robot_pkg