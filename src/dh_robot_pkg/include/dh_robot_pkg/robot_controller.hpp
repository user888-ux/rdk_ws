#pragma once

//第三方库
#include "ai_msgs/msg/perception_targets.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/message_info.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <kdl/frames.hpp>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
//用于调试
#include <iostream>
#define LOG(msg) std::cerr << msg << std::endl   // 使用cerr避免缓冲
//自己写的库
#include "dh_robot_pkg/serial_communicator.hpp"
#include "dh_robot_pkg/stereo_system.hpp"
// #include "dh_robot_pkg/dh_kinematics.hpp"

using PerceptionTargets = ai_msgs::msg::PerceptionTargets;

class RobotController : public rclcpp::Node {
public:
    RobotController();
    ~RobotController() = default;

private:
    // 串口通信器
    std::unique_ptr<SerialCommunicator> serial_;

    // 接收数据累积缓冲区
    std::vector<uint8_t> rx_buffer_;

    // 时间起点（节点启动时刻）
    std::chrono::steady_clock::time_point start_time_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    //推理消息订阅者
    rclcpp::Subscription<PerceptionTargets>::SharedPtr ai_subscription_;

    // 选择定时器还是回调
    bool mode;

    // 测距对象
    std::unique_ptr<StereoSystem> stereo_;

    // VOFA+ 帧格式常量
    static constexpr size_t FRAME_SIZE = 8;               // AA 01 04 %% 0D
    static constexpr uint32_t FRAME_TAIL = 0x7F800000;     // 正无穷浮点数的十六进制表示

    // 定时器函数
    void timer_callback();
    // 回调函数
    void ai_subscription_callback(const PerceptionTargets::SharedPtr msg);

    // 回调：处理接收到的原始字节流
    void onSerialData(const uint8_t* data, size_t len);

    // 解析累积缓冲区中的完整帧
    void parseFrames();

    // 处理一帧有效数据（pitch, yaw）
    void processFrame(const std::vector<float>& values);

    // 发送响应帧（解算后的两个角度）
    void sendResponse(float target_lu,float target_lv,float find_target,float target_dist);

    //定时发送帧
    void sendResponse();
    void timerCallback();

    //获取定时器还是回调模式
    bool getMode(){return mode;}
};
