// yolov5_detector.hpp
#pragma once

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <opencv2/opencv.hpp>
#include "dnn/hb_dnn.h"
#define LOG(msg) std::cout<<msg<<std::endl;

/**
 * @brief 基于地平线BPU的YOLOv5目标检测器，内部使用双线程流水线（推理+后处理）
 */
class YOLOv5Detector {
public:
    /**
     * @brief 构造函数，设置模型路径和检测参数
     * @param model_path  D-Robotics量化后模型文件(.bin)的路径
     * @param classes_num 模型输出的类别数量，默认80（COCO）
     * @param score_threshold 置信度阈值，默认0.25
     * @param nms_threshold   NMS时的IoU阈值，默认0.45
     * @param nms_top_k       NMS时考虑的最大候选框数，默认300
     */
    YOLOv5Detector(const std::string& model_path, int classes_num = 80,
                   float score_threshold = 0.25, float nms_threshold = 0.45, int nms_top_k = 300);

    /**
     * @brief 析构函数，释放模型及线程资源
     */
    ~YOLOv5Detector();

    /**
     * @brief 初始化模型，创建输出tensor，并启动工作线程
     * @return 成功返回true，失败返回false
     */
    bool init();

    /**
     * @brief 将一帧NV12图像提交到推理队列（非阻塞）
     * @param nv12_image 输入的NV12格式图像，需满足模型要求的分辨率及对齐要求
     * @param image_id   帧的标识ID，用于异步获取结果
     */
    void submitImage(const cv::Mat& nv12_image, int image_id);

    /**
     * @brief 获取指定ID图像的检测结果，若结果未生成则返回false
     * @param image_id  帧标识ID
     * @param bboxes    输出边界框列表，每个框为(x, y, width, height)
     * @param scores    输出每个框的置信度
     * @param class_ids 输出每个框的类别索引
     * @return 成功获取到结果返回true，否则返回false
     */
    bool getResult(int image_id, std::vector<cv::Rect2d>& bboxes,
                   std::vector<float>& scores, std::vector<int>& class_ids);

private:
    /**
     * @brief 推理线程入口，循环等待图像到来，执行BPU推理
     */
    void inferenceThread();

    /**
     * @brief 后处理线程入口，循环等待推理完成，执行解码与NMS
     */
    void postprocessThread();

    /**
     * @brief 执行一次BPU推理（由推理线程调用）
     * @param nv12_image 输入NV12图像
     * @param output_data 输出各层特征图的数据指针列表（指向BPU内存）
     */
    void infer(const cv::Mat& nv12_image, std::vector<float*>& output_data);

    // 后处理中使用的工具函数
    void decodeOutput(const std::vector<float*>& output_data,
                      std::vector<std::vector<cv::Rect2d>>& bboxes,
                      std::vector<std::vector<float>>& scores);

    void applyNMS(std::vector<std::vector<cv::Rect2d>>& bboxes,
                  std::vector<std::vector<float>>& scores,
                  std::vector<cv::Rect2d>& final_boxes,
                  std::vector<float>& final_scores,
                  std::vector<int>& final_class_ids);

    // 模型相关
    hbPackedDNNHandle_t packed_dnn_handle_; ///< 打包模型句柄
    hbDNNHandle_t dnn_handle_;              ///< 单个模型句柄
    std::string model_path_;                ///< 模型文件路径
    int classes_num_;                       ///< 类别数
    float score_threshold_;                 ///< 置信度阈值
    float nms_threshold_;                   ///< NMS IoU阈值
    int nms_top_k_;                         ///< NMS最大候选数
    int input_h_, input_w_;                 ///< 模型输入的高和宽
    std::vector<int> output_order_;         ///< 输出特征图的顺序映射

    /// 推理输出tensor，推理线程使用（固定内存，重复利用）
    std::vector<hbDNNTensor> output_tensors_;

    // 线程相关
    std::thread inference_thread_;          ///< 推理线程对象
    std::thread postprocess_thread_;        ///< 后处理线程对象
    std::mutex mtx_;                        ///< 互斥锁，保护任务队列和结果存储
    std::condition_variable cv_infer_;      ///< 推理线程条件变量
    std::condition_variable cv_post_;       ///< 后处理线程条件变量
    bool stop_ = false;                     ///< 线程退出标志

    // 任务队列
    struct Task {
        cv::Mat image; ///< 图像数据（NV12）
        int id;        ///< 帧ID
        std::vector<std::vector<float>> output_copies;// 拷贝后的特征图数据
    };
    std::queue<Task> inference_queue_;      ///< 待推理的图像队列
    std::queue<Task> postprocess_queue_;    ///< 待后处理的图像队列

    // 后处理结果存储
    struct Result {
        std::vector<cv::Rect2d> bboxes;
        std::vector<float> scores;
        std::vector<int> class_ids;
    };
    std::map<int, Result> results_;         ///< 已完成后处理的结果，键为帧ID

    // anchors 分组（小、中、大）
    std::vector<std::pair<float, float>> small_anchors_;
    std::vector<std::pair<float, float>> medium_anchors_;
    std::vector<std::pair<float, float>> large_anchors_;

    // 类别名称（可选，如需在检测器内部保存）
    std::vector<std::string> class_names_;
};