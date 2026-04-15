#include "dh_robot_pkg/robot_model.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
//纯进行机器人的选型 并根据选型配置参数
namespace dh_robot_pkg {

RobotModel::RobotModel() {
    // 默认设置一个简单的机器人模型
    setupSCARAModel();
}

//输入YAML文件路径并加载，成功就返回 true
bool RobotModel::loadParametersFromFile(const std::string& file_path) {
    try {
        YAML::Node config = YAML::LoadFile(file_path);
        auto dh_params_node = config["robot_controller"]["dh_parameters"];
        
        if (!dh_params_node) {
            std::cerr << "No DH parameters found in config file" << std::endl;
            return false;
        }
        
        //初始化新的参数列表
        std::vector<DHParameters> new_params;
        for (const auto& param : dh_params_node) {
            if (param.size() == 4) {
                DHParameters dh_param;
                dh_param.a = param[0].as<double>();//连杆长度
                dh_param.alpha = param[1].as<double>();//连杆扭角
                dh_param.d = param[2].as<double>();//连杆偏距
                dh_param.theta = param[3].as<double>();//关节角度偏移
                new_params.push_back(dh_param);
            }
        }
        
        if (!new_params.empty()) {
            current_parameters_ = new_params;
            kinematics_ = std::make_shared<DHKinematics>(current_parameters_);
            return true;
        }
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML parsing error: " << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error loading parameters: " << e.what() << std::endl;
    }
    
    return false;
}

void RobotModel::setupSCARAModel() {
    // SCARA机器人D-H参数示例
    current_parameters_ = {
        {0.0, 0.0, 0.3, 0.0},      // 关节1: 旋转关节
        {0.25, 0.0, 0.0, 0.0},     // 关节2: 旋转关节  
        {0.0, 0.0, -0.1, 0.0},     // 关节3: 平移关节
        {0.0, 0.0, 0.05, 0.0}      // 末端执行器
    };
    kinematics_ = std::make_shared<DHKinematics>(current_parameters_);
}

void RobotModel::setupArticulatedModel() {
    // 关节型机器人D-H参数示例
    current_parameters_ = {
        {0.0, M_PI/2, 0.1, 0.0},   // 关节1: 旋转，绕Z轴
        {0.2, 0.0, 0.0, 0.0},      // 关节2: 旋转，在XZ平面
        {0.15, 0.0, 0.0, 0.0},     // 关节3: 旋转，在XZ平面
        {0.0, M_PI/2, 0.05, 0.0},  // 关节4: 旋转，绕Z轴
        {0.0, 0.0, 0.03, 0.0}      // 末端执行器
    };
    kinematics_ = std::make_shared<DHKinematics>(current_parameters_);
}

void RobotModel::setupCartesianModel() {
    // 直角坐标机器人D-H参数示例
    current_parameters_ = {
        {0.0, 0.0, 0.0, 0.0},      // X轴: 平移关节
        {0.0, -M_PI/2, 0.0, 0.0},  // Y轴: 平移关节
        {0.0, 0.0, 0.0, 0.0},      // Z轴: 平移关节
        {0.0, 0.0, 0.05, 0.0}      // 末端执行器
    };
    kinematics_ = std::make_shared<DHKinematics>(current_parameters_);
}

//关节数量获取
size_t RobotModel::getNumberOfJoints() const {
    if (!kinematics_) return 0;
    // 对于SCARA模型，我们有3个关节
    // 关节1：旋转，关节2：旋转，关节3：平移
    return 3;//注意: 这里硬编码返回3，实际应该根据D-H参数动态计算。
}

} // namespace dh_robot_pkg