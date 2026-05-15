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
#include "dh_robot_pkg/serial_communicator.hpp"
#include "dh_robot_pkg/dh_kinematics.hpp"

class RobotController : public rclcpp::Node {
public:
    RobotController();
    ~RobotController() = default;

private:
    // 串口通信器
    std::unique_ptr<SerialCommunicator> serial_;

    // 接收数据累积缓冲区
    std::vector<uint8_t> rx_buffer_;

    // VOFA+ 帧格式常量
    static constexpr size_t FRAME_SIZE = 8;               // AA 01 04 %% 0D
    static constexpr uint32_t FRAME_TAIL = 0x7F800000;     // 正无穷浮点数的十六进制表示

    // 回调：处理接收到的原始字节流
    void onSerialData(const uint8_t* data, size_t len);

    // 解析累积缓冲区中的完整帧
    void parseFrames();

    // 处理一帧有效数据（pitch, yaw）
    void processFrame(const std::vector<float>& values);

    // 发送响应帧（解算后的两个角度）
    void sendResponse(float new_pitch, float new_yaw);
};