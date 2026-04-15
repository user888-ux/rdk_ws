#pragma once

#include "dh_kinematics.hpp"
#include <memory>

namespace dh_robot_pkg {

/**
 * @brief 机器人模型管理器
 */
class RobotModel {
public:
    RobotModel();
    
    /**
     * @brief 从YAML文件加载D-H参数
     */
    bool loadParametersFromFile(const std::string& file_path);
    
    //模型设置接口
    /**
     * @brief 设置预定义的机器人模型
     */
    void setupSCARAModel();      // SCARA机器人
    void setupArticulatedModel(); // 关节型机器人
    void setupCartesianModel();  // 直角坐标机器人
    
    //数据访问接口
    /**
     * @brief 获取运动学计算器
     */
    std::shared_ptr<DHKinematics> getKinematics() const{return kinematics_;}
    
    /**
     * @brief 获取关节数量
     */
    size_t getNumberOfJoints() const;

    //数据访问接口
    /**
     * @brief 运动学对象管理
     */
    std::vector<DHParameters> getDHParameters() const { return current_parameters_; }

private:
    std::shared_ptr<DHKinematics> kinematics_;
    std::vector<DHParameters> current_parameters_;
};

} // namespace dh_robot_pkg