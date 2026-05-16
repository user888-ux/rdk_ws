#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "camera_detector/detector_controller.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // 直接获取模型路径
    std::string package_share = ament_index_cpp::get_package_share_directory("camera_detector");
    std::string model_path = package_share + "/models/yolov5n_tag_v7.0_detect_640x640_bayese_nv12.bin";

    // 创建节点，参数可改为从launch文件或命令行传入
    auto node = std::make_shared<DetectorController>(
        "camera_detector_node",
        model_path, // 模型路径
        80,     // 类别数
        0.25,   // 置信度阈值
        0.45,   // NMS IoU阈值
        300,    // NMS TopK
        "camera_link"
    );

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// /root/rdk_ws/build/camera_detector/camera_detector_node
// /root/rdk_ws/