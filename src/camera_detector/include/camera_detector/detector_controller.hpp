// detector_controller.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include "yolov5_detector.hpp"
#include <opencv2/opencv.hpp>
// #include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include <string>
#include <memory>

#define LOG(msg) std::cout<<msg<<std::endl;

class DetectorController : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数，初始化检测器、订阅与发布
     * @param node_name 节点名称
     * @param model_path 模型文件路径
     * @param classes_num 类别数
     * @param score_thresh 置信度阈值
     * @param nms_thresh NMS阈值
     * @param nms_top_k NMS前K个候选
     * @param frame_id 坐标系的frame_id，用于消息头
     */
    DetectorController(
        const std::string & node_name,
        const std::string & model_path,
        int classes_num = 80,
        float score_thresh = 0.25f,
        float nms_thresh = 0.45f,
        int nms_top_k = 300,
        const std::string & frame_id = "camera");

    ~DetectorController();

private:
    /// 图像订阅回调，将NV12图像转为cv::Mat并提交
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    /// 定时器回调，轮询检测结果并发布
    void processResults();

    /// 根据检测结果生成渲染图像（BGR）
    cv::Mat renderImage(const cv::Mat & bgr_image,
                        const std::vector<cv::Rect2d> & bboxes,
                        const std::vector<float> & scores,
                        const std::vector<int> & class_ids);

    /// 生成Detection2DArray消息
    vision_msgs::msg::Detection2DArray
    generateDetectionMsg(
        const std_msgs::msg::Header & header,
        const std::vector<cv::Rect2d> & bboxes,
        const std::vector<float> & scores,
        const std::vector<int> & class_ids);

    std::string get_model_path();

    // ROS 接口
    rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_pub_;
    rclcpp::TimerBase::SharedPtr result_timer_;

    // 检测器实例
    std::unique_ptr<YOLOv5Detector> detector_;

    // 参数
    std::string frame_id_;
    int next_image_id_ = 0;   ///< 帧ID，递增

    // 当前等待渲染的原始BGR图像（用于绘制，需从订阅时保存）
    cv::Mat current_bgr_image_;

    //互斥锁
    std::mutex map_mutex_;                       ///< 保护 bgr_map_ 的互斥锁
    std::map<int, cv::Mat> bgr_map_;            ///< 帧ID -> 原始BGR图像的映射（用于渲染）
};