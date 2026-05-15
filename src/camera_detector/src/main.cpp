// main.cpp
#include "rclcpp/rclcpp.hpp"
#include "camera_detector/detector_controller.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // 创建节点，参数可改为从launch文件或命令行传入
    auto node = std::make_shared<DetectorController>(
        "detector_controller",
        "../../models/yolov5s_tag_v2.0_detect_640x640_bayese_nv12.bin", // 模型路径
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