#include "dh_robot_pkg/dh_kinematics.hpp"
#include <stdexcept>
#include <cmath>
//纯数学库，用来计算末端位姿
namespace dh_robot_pkg {

DHKinematics::DHKinematics() : parameters_initialized_(false) {}

DHKinematics::DHKinematics(const std::vector<DHParameters>& dh_params) 
    : dh_parameters_(dh_params), parameters_initialized_(true) {}

//结构体数组包含设置参数
void DHKinematics::setDHParameters(const std::vector<DHParameters>& dh_params) {
    dh_parameters_ = dh_params;
    parameters_initialized_ = true;
}

// 输入：关节角度向量
// 输出：末端执行器的位姿（KDL::Frame）
KDL::Frame DHKinematics::forwardKinematics(const std::vector<double>& joint_angles) {
    if (!parameters_initialized_) {
        throw std::runtime_error("D-H parameters not initialized");
    }
    
    //验证输入的关节数，如果<3则抛出错误
    validateJointAngles(joint_angles);
    
    //用官方的函数 Identity()初始化计算结果
    KDL::Frame result = KDL::Frame::Identity();
    
    // 只处理实际关节数量（对于SCARA是3个关节+1个末端执行器）
    size_t num_joints = joint_angles.size();
    for (size_t i = 0; i < dh_parameters_.size(); ++i) {
        // 对于旋转关节：joint_value 是关节的旋转角度（弧度制）
        // 对于移动关节：joint_value 是关节的线性位移（米）
        double joint_value = (i < num_joints) ? joint_angles[i] : 0.0;
        //核心自定义函数：计算并返回变换矩阵
        //D-H变换公式：Transform = RotZ(θ) × TransZ(d) × TransX(a) × RotX(α)（4个矩阵连乘，每个都代表一次变换）
        //D-H规定的每一次的变换次序为：先绕x轴旋转alpha角，再向x轴平移a单位长度，再绕z轴旋转theta角，最后再沿着轴平移d距离
        //对应齐次变换矩阵（这个结果本质是4个矩阵相乘的结果）
        // [ cosθ    -sinθ·cosα    sinθ·sinα    a·cosθ ]
        // [ sinθ     cosθ·cosα   -cosθ·sinα    a·sinθ ]
        // [  0         sinα         cosα         d    ]
        // [  0          0            0          1     ]
        KDL::Frame transform = calculateDHTransform(dh_parameters_[i], joint_value);
        result = result * transform;
    }
    
    return result;
}

//通过关节角度向量计算对应变换矩阵
KDL::Frame DHKinematics::calculateDHTransform(const DHParameters& dh, double joint_value) {
    double theta = dh.theta + joint_value;//dh.theta 为初始偏移
    double ct = cos(theta);
    double st = sin(theta);
    double ca = cos(dh.alpha);
    double sa = sin(dh.alpha);
    
    KDL::Frame transform;//初始化结果矩阵
    
    // 设置3×3旋转矩阵
    transform.M = KDL::Rotation(
        ct,    -st * ca,   st * sa,
        st,     ct * ca,  -ct * sa,
        0.0,    sa,        ca
    );
    
    // 设置3×1位置向量
    transform.p = KDL::Vector(
        dh.a * ct,
        dh.a * st,
        dh.d
    );
    
    return transform;//合起来本质上是4阶矩阵
}

//输入关节角度向量，内部执行forwardKinematics(joint_angles)后
//得到末端执行器的位姿(KDL::Frame),而后将它分解为位置向量和旋转向量
void DHKinematics::getEndEffectorPose(const std::vector<double>& joint_angles, 
                                     KDL::Vector& position, KDL::Rotation& orientation) {
    KDL::Frame end_effector = forwardKinematics(joint_angles);
    position = end_effector.p;
    orientation = end_effector.M;
}

void DHKinematics::validateJointAngles(const std::vector<double>& joint_angles) {
    if (joint_angles.size() < 3) {  // SCARA需要至少3个关节
        throw std::runtime_error("Insufficient joint angles provided. Expected 3 joints, got " + 
                                std::to_string(joint_angles.size()));
    }
}

} // namespace dh_robot_pkg