#include "dh_robot_pkg/robot_controller.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // 直接创建节点实例，而不是使用组件
    auto node = std::make_shared<RobotController>();
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    
    try {
        RCLCPP_INFO(node->get_logger(), "Starting D-H Robot Controller");
        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}