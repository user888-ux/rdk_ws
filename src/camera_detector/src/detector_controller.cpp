// detector_controller.cpp
#include "camera_detector/detector_controller.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

DetectorController::DetectorController(
    const std::string & node_name,
    const std::string & model_path,
    int classes_num,
    float score_thresh,
    float nms_thresh,
    int nms_top_k,
    const std::string & frame_id)
    : Node(node_name), frame_id_(frame_id)
{
    // 初始化检测器
    detector_ = std::make_unique<YOLOv5Detector>(
        model_path, classes_num, score_thresh, nms_thresh, nms_top_k);
    if (!detector_->init()) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize detector");
        rclcpp::shutdown();
    }

    // 订阅NV12图像话题
    // 假设发布者为官方硬件节点，编码格式为 "nv12"
    image_sub_ = create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
        "/nv12_img", 10,
        std::bind(&DetectorController::imageCallback, this, std::placeholders::_1));

    // 发布渲染后的BGR图像
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("/detection_img", 10);

    // 发布检测结果（检测框、类别、分数）
    det_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>("/detections", 10);

    // 定时器每10ms轮询一次检测结果
    result_timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&DetectorController::processResults, this));

    RCLCPP_INFO(get_logger(), "DetectorController ready");
}

DetectorController::~DetectorController()
{
    // 显式释放检测器（确保线程在ROS析构前停止）
    detector_.reset();
}

void DetectorController::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    LOG("触发了图像回调");
    // 将ROS Image消息转为cv::Mat (NV12格式)
    // 注意：NV12是单通道图像，大小为 height * 1.5 × width
    if (msg->encoding != "nv12") {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                              "Only nv12 encoding is supported");
        return;
    }
    // 构建cv::Mat，直接引用数据，不拷贝（发布者保证生命周期）
    cv::Mat nv12_img(msg->height * 3 / 2, msg->width, CV_8UC1,
                     const_cast<unsigned char *>(msg->data.data()));

    // 为渲染保存一份BGR版本的图像（需要转换，此处我们保存原始NV12，
    // 渲染时再转为BGR，或直接保存回调时的原始BGR图像）
    // 由于输入为NV12，我们需先转为BGR用于绘图
    cv::Mat bgr_img;
    cv::cvtColor(nv12_img, bgr_img, cv::COLOR_YUV2BGR_NV12);
    current_bgr_image_ = bgr_img.clone();  // 拷贝一份，避免数据失效

    // 向检测器提交NV12图像（需深拷贝，避免ROS消息内存被释放）
    cv::Mat nv12_copy = nv12_img.clone();
    int image_id = next_image_id_++;
    detector_->submitImage(nv12_copy, image_id);

    // 存储 image_id -> 原始BGR图的映射（此处简单保存在map中）
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        bgr_map_[image_id] = bgr_img.clone();
    }
    LOG("完成回调");
}

void DetectorController::processResults()
{
    // 尝试获取已完成的结果
    static int processed_id = 0;  // 仅示例，实际应取最早完成的id
    std::vector<cv::Rect2d> bboxes;
    std::vector<float> scores;
    std::vector<int> class_ids;

    while (detector_->getResult(processed_id, bboxes, scores, class_ids)) {
        // 渲染图像
        cv::Mat bgr;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            auto it = bgr_map_.find(processed_id);
            if (it != bgr_map_.end()) {
                bgr = it->second;
                bgr_map_.erase(it);
            } else {
                processed_id++;
                continue;
            }
        }

        cv::Mat rendered = renderImage(bgr, bboxes, scores, class_ids);

        // 发布渲染图像
        std_msgs::msg::Header header;
        header.stamp = now();
        header.frame_id = frame_id_;
        sensor_msgs::msg::Image::SharedPtr img_msg =
            cv_bridge::CvImage(header, "bgr8", rendered).toImageMsg();
        image_pub_->publish(*img_msg);

        // 发布检测结果
        auto det_msg = generateDetectionMsg(header, bboxes, scores, class_ids);
        det_pub_->publish(det_msg);

        processed_id++;
    }
}

cv::Mat DetectorController::renderImage(
    const cv::Mat & bgr_image,
    const std::vector<cv::Rect2d> & bboxes,
    const std::vector<float> & scores,
    const std::vector<int> & class_ids)
{
    cv::Mat img = bgr_image.clone();
    // 这里仅示例绘制矩形，实际可加入类别名、颜色等
    for (size_t i = 0; i < bboxes.size(); i++) {
        cv::rectangle(img, bboxes[i], cv::Scalar(0, 255, 0), 2);
        std::string text = std::to_string(class_ids[i]) + ": " +
                           std::to_string(static_cast<int>(scores[i] * 100)) + "%";
        cv::putText(img, text,
                    cv::Point(bboxes[i].x, bboxes[i].y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
    return img;
}

vision_msgs::msg::Detection2DArray
DetectorController::generateDetectionMsg(
    const std_msgs::msg::Header & header,
    const std::vector<cv::Rect2d> & bboxes,
    const std::vector<float> & scores,
    const std::vector<int> & class_ids)
{
    vision_msgs::msg::Detection2DArray array_msg;
    array_msg.header = header;
    for (size_t i = 0; i < bboxes.size(); i++) {
        vision_msgs::msg::Detection2D det;
        // 通过 Pose2D 的 x, y 字段设置中心点坐标
        det.bbox.center.position.x = bboxes[i].x + bboxes[i].width / 2.0;
        det.bbox.center.position.y = bboxes[i].y + bboxes[i].height / 2.0;
        det.bbox.size_x = bboxes[i].width;
        det.bbox.size_y = bboxes[i].height;
        det.results.resize(1);
        det.results[0].hypothesis.class_id = std::to_string(class_ids[i]);
        det.results[0].hypothesis.score = scores[i];
        array_msg.detections.push_back(det);
    }
    return array_msg;
}