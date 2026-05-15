#pragma once

#include <kdl/frames.hpp>
#include <vector>
#include <array>

namespace dh_robot_pkg {

/**
 * @brief D-H参数结构体
 */
struct DHParameters {
    double a;      // 连杆长度
    double alpha;  // 连杆扭转角
    double d;      // 连杆偏距
    double theta;  // 关节角
};

/**
 * @brief D-H运动学计算类
 */
class DHKinematics {
public:
    DHKinematics();
    explicit DHKinematics(const std::vector<DHParameters>& dh_params);
    
    /**
     * @brief 设置D-H参数
     */
    void setDHParameters(const std::vector<DHParameters>& dh_params);
    
    /**
     * @brief 计算正向运动学
     * @param joint_angles 关节角度向量
     * @return 末端执行器的变换矩阵
     */
    KDL::Frame forwardKinematics(const std::vector<double>& joint_angles);
    
    /**
     * @brief 计算单个D-H变换矩阵
     */
    KDL::Frame calculateDHTransform(const DHParameters& dh, double joint_value);
    
    /**
     * @brief 获取末端位姿
     */
    void getEndEffectorPose(const std::vector<double>& joint_angles, 
                           KDL::Vector& position, KDL::Rotation& orientation);
    
    /**
     * @brief 获取雅可比矩阵（可选实现）
     */
    // KDL::Jacobian calculateJacobian(const std::vector<double>& joint_angles);

private:
    /**
     * @brief  D-H参数结构体，包含固定的连杆参数
     */
    std::vector<DHParameters> dh_parameters_;
    bool parameters_initialized_;
    
    void validateJointAngles(const std::vector<double>& joint_angles);
};

} // namespace dh_robot_pkg