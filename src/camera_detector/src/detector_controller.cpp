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
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
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

// void DetectorController::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
// {
//     cv::Mat nv12_img;

//     if (msg->encoding == "nv12") {
//         // 直接使用，构造 CV_8UC1 单通道矩阵（高度为 3/2 * height）
//         nv12_img = cv::Mat(msg->height * 3 / 2, msg->width, CV_8UC1,
//                            const_cast<unsigned char*>(msg->data.data()));
//     }
//     else {
//         // 将 BGR 消息转为 cv::Mat
//         cv::Mat bgr(msg->height, msg->width, CV_8UC3,
//                     const_cast<unsigned char*>(msg->data.data()));
//         // 调用转换函数（需提前从模型获取 input_h_, input_w_，并决定是否 letterbox）
//         nv12_img = bgrToNV12(bgr, msg->height * 3 / 2, msg->width, true);
//     }
//     // else {
//     //     RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
//     //                           "Unsupported image encoding: %s", msg->encoding.c_str());
//     //     return;
//     // }

//     // 深拷贝一份 NV12 数据，防止 ROS 消息内存被释放
//     cv::Mat nv12_copy = nv12_img.clone();
//     int image_id = next_image_id_++;
//     detector_->submitImage(nv12_copy, image_id);
// }

void DetectorController::processResults()
{
    // 尝试获取已完成的结果
    static int processed_id = 0;  // 仅示例，实际应取最早完成的id
    std::vector<cv::Rect2d> bboxes;
    std::vector<float> scores;
    std::vector<int> class_ids;

    while (detector_->getResult(processed_id, bboxes, scores, class_ids)) {
        // 渲染图像
        LOG("正在渲染图像");
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
    cv::Mat canvas = bgr_image.clone();

    // 类别名称映射（可与检测器内部保持一致）
    static const std::vector<std::string> class_names = {
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
        "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
        "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
        "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
        "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
        "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
        "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
        "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
        "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
        "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
        "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
        "toothbrush"
    };

    // 为每个类别生成固定颜色
    static std::vector<cv::Scalar> colors;
    if (colors.empty()) {
        for (size_t i = 0; i < class_names.size(); ++i) {
            cv::Scalar color(cv::randu<int>() % 256, cv::randu<int>() % 256, cv::randu<int>() % 256);
            colors.push_back(color);
        }
    }

    for (size_t i = 0; i < bboxes.size(); ++i) {
        const cv::Rect2d& box = bboxes[i];
        int cls = class_ids[i];
        cv::Scalar color = (cls < static_cast<int>(colors.size())) ? colors[cls] : cv::Scalar(0, 255, 0);

        // 绘制矩形框
        cv::rectangle(canvas, cv::Point(box.x, box.y),
                      cv::Point(box.x + box.width, box.y + box.height),
                      color, 2);

        // 构造标签文本
        std::string label;
        if (cls >= 0 && cls < static_cast<int>(class_names.size())) {
            label = class_names[cls];
        } else {
            label = "cls_" + std::to_string(cls);
        }
        label += ": " + std::to_string(static_cast<int>(scores[i] * 100)) + "%";

        // 计算文本背景框大小
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        cv::Point label_origin(box.x, box.y - 5);
        if (label_origin.y - text_size.height < 0) {
            label_origin.y = box.y + text_size.height + 5;
        }
        cv::rectangle(canvas,
                      cv::Point(label_origin.x, label_origin.y - text_size.height),
                      cv::Point(label_origin.x + text_size.width, label_origin.y + baseline),
                      color, -1);
        cv::putText(canvas, label, label_origin,
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }

    return canvas;
}

vision_msgs::msg::Detection2DArray
DetectorController::generateDetectionMsg(
    const std_msgs::msg::Header & header,
    const std::vector<cv::Rect2d> & bboxes,
    const std::vector<float> & scores,
    const std::vector<int> & class_ids)
{
    LOG("正在获取结果");
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

cv::Mat DetectorController::bgrToNV12(const cv::Mat& bgr_img,
                                      int target_h,
                                      int target_w,
                                      bool use_letterbox)
{
    // 1. 预处理：缩放 + 可能的 letterbox
    cv::Mat resized;
    float scale = 1.0f;
    int top = 0, left = 0;   // 偏移量（仅 letterbox 时有意义）

    if (use_letterbox) {
        // 保持宽高比的 letterbox
        scale = std::min(static_cast<float>(target_h) / bgr_img.rows,
                         static_cast<float>(target_w) / bgr_img.cols);
        int new_w = static_cast<int>(bgr_img.cols * scale);
        int new_h = static_cast<int>(bgr_img.rows * scale);
        cv::resize(bgr_img, resized, cv::Size(new_w, new_h));

        top  = (target_h - new_h) / 2;
        left = (target_w - new_w) / 2;
        int bottom = target_h - new_h - top;
        int right  = target_w - new_w - left;
        cv::copyMakeBorder(resized, resized,
                           top, bottom, left, right,
                           cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    } else {
        // 直接缩放到目标尺寸
        cv::resize(bgr_img, resized, cv::Size(target_w, target_h));
    }

    // 2. 颜色空间转换：BGR -> YUV_I420
    cv::Mat yuv_i420;
    cv::cvtColor(resized, yuv_i420, cv::COLOR_BGR2YUV_I420);

    // 3. 手动打包为 NV12 格式 (Y 平面 + UV 交错平面)
    cv::Mat nv12(target_h * 3 / 2, target_w, CV_8UC1);
    uint8_t* p_nv12_y  = nv12.data;
    uint8_t* p_nv12_uv = p_nv12_y + target_h * target_w;

    const uint8_t* p_y = yuv_i420.data;
    const uint8_t* p_u = p_y + target_h * target_w;
    const uint8_t* p_v = p_u + (target_h / 2) * (target_w / 2);

    // 复制 Y 平面
    std::memcpy(p_nv12_y, p_y, target_h * target_w);

    // 交错复制 UV 平面
    size_t uv_count = static_cast<size_t>(target_h / 2) * (target_w / 2);
    for (size_t i = 0; i < uv_count; ++i) {
        p_nv12_uv[2 * i]     = p_u[i];
        p_nv12_uv[2 * i + 1] = p_v[i];
    }

    return nv12;
}