#include "camera_detector/yolov5_detector.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstring>

YOLOv5Detector::YOLOv5Detector(const std::string& model_path, int classes_num,
                               float score_threshold, float nms_threshold, int nms_top_k)
    : model_path_(model_path), classes_num_(classes_num),
      score_threshold_(score_threshold), nms_threshold_(nms_threshold), nms_top_k_(nms_top_k) {}

bool YOLOv5Detector::init() {
    //0. 初始化 anchors，通常与模型训练一致
    small_anchors_  = {{10, 13}, {16, 30}, {33, 23}};
    medium_anchors_ = {{30, 61}, {62, 45}, {59, 119}};
    large_anchors_  = {{116, 90}, {156, 198}, {373, 326}};

    // 1. 加载模型
    const char* model_file = model_path_.c_str();
    if (hbDNNInitializeFromFiles(&packed_dnn_handle_, &model_file, 1) != 0) {
        std::cerr << "Failed to load model" << std::endl;
        return false;
    }

    // 2. 获取模型信息
    const char** name_list;
    int model_count;
    hbDNNGetModelNameList(&name_list, &model_count, packed_dnn_handle_);
    hbDNNGetModelHandle(&dnn_handle_, packed_dnn_handle_, name_list[0]);

    // 3. 获取输入属性
    hbDNNTensorProperties input_prop;
    hbDNNGetInputTensorProperties(&input_prop, dnn_handle_, 0);
    input_h_ = input_prop.validShape.dimensionSize[2];
    input_w_ = input_prop.validShape.dimensionSize[3];
    // 注意：实际开发中还需根据input_prop.validShape.numDimensions和tensorType校验是NV12输入

    // 4. 获取输出属性与内存分配
    int output_count;
    hbDNNGetOutputCount(&output_count, dnn_handle_);
    output_tensors_.resize(output_count);
    for (int i = 0; i < output_count; ++i) {
        hbDNNGetOutputTensorProperties(&output_tensors_[i].properties, dnn_handle_, i);
        int size = output_tensors_[i].properties.alignedByteSize;
        hbSysAllocCachedMem(&output_tensors_[i].sysMem[0], size);
    }

    // 5. 确定输出顺序
    output_order_ = {0, 1, 2}; // 默认顺序，实际需要根据特征图尺寸匹配
    // 此处省略具体的匹配逻辑，可参考原代码

    // 6. 启动工作线程
    inference_thread_ = std::thread(&YOLOv5Detector::inferenceThread, this);
    postprocess_thread_ = std::thread(&YOLOv5Detector::postprocessThread, this);
    return true;
}

YOLOv5Detector::~YOLOv5Detector() {
    stop_ = true;
    cv_infer_.notify_all();
    cv_post_.notify_all();
    if (inference_thread_.joinable()) inference_thread_.join();
    if (postprocess_thread_.joinable()) postprocess_thread_.join();
    
    for (auto& tensor : output_tensors_) {
        hbSysFreeMem(&tensor.sysMem[0]);
    }
    hbDNNRelease(packed_dnn_handle_);
}

void YOLOv5Detector::submitImage(const cv::Mat& nv12_image, int image_id) {
    LOG("成功提交图像到任务队列");
    std::unique_lock<std::mutex> lock(mtx_);
    inference_queue_.push({nv12_image.clone(), image_id,{}});
    cv_infer_.notify_one();
}

bool YOLOv5Detector::getResult(int image_id, std::vector<cv::Rect2d>& bboxes,
                               std::vector<float>& scores, std::vector<int>& class_ids) {
    std::unique_lock<std::mutex> lock(mtx_);
    auto it = results_.find(image_id);
    if (it != results_.end()) {
        bboxes = it->second.bboxes;
        scores = it->second.scores;
        class_ids = it->second.class_ids;
        results_.erase(it);
        return true;
    }
    return false;
}

void YOLOv5Detector::inferenceThread() {
    while (!stop_) {
        LOG("开始推理");
        Task task;
        {
            std::unique_lock<std::mutex> lock(mtx_);
            cv_infer_.wait(lock, [this] { return !inference_queue_.empty() || stop_; });
            if (stop_ && inference_queue_.empty()) break;
            task = inference_queue_.front();
            inference_queue_.pop();
        }
        
        // 执行BPU推理
        std::vector<float*> output_data;
        infer(task.image, output_data);
        
        // 将推理结果传递给后处理队列
        std::unique_lock<std::mutex> lock(mtx_);
        // 注意：这里的output_data是BPU内存指针，实际传递时需要考虑内存生命周期
        // 简化示例中直接将指针存入任务，实际需要更安全的管理方式
        postprocess_queue_.push({task.image, task.id,{}});
        cv_post_.notify_one();
    }
}

void YOLOv5Detector::postprocessThread() {
    while (!stop_) {
        Task task;
        {
            std::unique_lock<std::mutex> lock(mtx_);
            cv_post_.wait(lock, [this] { return !postprocess_queue_.empty() || stop_; });
            if (stop_ && postprocess_queue_.empty()) break;
            task = std::move(postprocess_queue_.front());
            postprocess_queue_.pop();
        }

        // 准备 feature map 数据指针
        std::vector<float*> output_ptrs;
        for (auto& copy : task.output_copies) {
            output_ptrs.push_back(copy.data());
        }

        // 解码所有检测框
        std::vector<std::vector<cv::Rect2d>> bboxes_per_class(classes_num_);
        std::vector<std::vector<float>> scores_per_class(classes_num_);
        decodeOutput(output_ptrs, bboxes_per_class, scores_per_class);

        // 执行 NMS
        std::vector<cv::Rect2d> final_boxes;
        std::vector<float> final_scores;
        std::vector<int> final_class_ids;
        applyNMS(bboxes_per_class, scores_per_class,
                 final_boxes, final_scores, final_class_ids);

        // 存储结果
        {
            std::lock_guard<std::mutex> lock(mtx_);
            Result res;
            res.bboxes = final_boxes;
            res.scores = final_scores;
            res.class_ids = final_class_ids;
            results_[task.id] = res;
        }
    }
}

void YOLOv5Detector::decodeOutput(const std::vector<float*>& output_data,
                                  std::vector<std::vector<cv::Rect2d>>& bboxes,
                                  std::vector<std::vector<float>>& scores) {
    // output_data 的顺序已根据 output_order_ 调整，即 output_data[0] 为小特征图，[1] 为中，[2] 为大
    float conf_thres_raw = -std::log(1.0f / score_threshold_ - 1.0f);
    std::vector<std::pair<float, float>>* anchor_sets[3] = {
        &small_anchors_, &medium_anchors_, &large_anchors_
    };
    int strides[3] = {8, 16, 32};
    int feature_h[3], feature_w[3];
    for (int i = 0; i < 3; ++i) {
        feature_h[i] = input_h_ / strides[i];
        feature_w[i] = input_w_ / strides[i];
    }

    for (int idx = 0; idx < 3; ++idx) {
        float* raw = output_data[idx];
        int h = feature_h[idx];
        int w = feature_w[idx];
        auto& anchors = *anchor_sets[idx];
        int stride = strides[idx];

        for (int row = 0; row < h; ++row) {
            for (int col = 0; col < w; ++col) {
                for (auto& anchor : anchors) {
                    float* cur = raw;
                    raw += (5 + classes_num_);

                    // 置信度过滤
                    if (cur[4] < conf_thres_raw) continue;

                    // 类别概率及最高类别
                    int max_cls_id = 5;
                    float max_cls_val = cur[5];
                    for (int c = 6; c < 5 + classes_num_; ++c) {
                        if (cur[c] > max_cls_val) {
                            max_cls_val = cur[c];
                            max_cls_id = c;
                        }
                    }
                    float score = 1.0f / (1.0f + std::exp(-cur[4])) *
                                  1.0f / (1.0f + std::exp(-max_cls_val));
                    if (score < score_threshold_) continue;

                    int cls_id = max_cls_id - 5;

                    // 解码边界框
                    float cx = (1.0f / (1.0f + std::exp(-cur[0])) * 2.0f - 0.5f + col) * stride;
                    float cy = (1.0f / (1.0f + std::exp(-cur[1])) * 2.0f - 0.5f + row) * stride;
                    float w_box = std::pow(1.0f / (1.0f + std::exp(-cur[2])) * 2.0f, 2) * anchor.first;
                    float h_box = std::pow(1.0f / (1.0f + std::exp(-cur[3])) * 2.0f, 2) * anchor.second;
                    float x = cx - w_box / 2.0f;
                    float y = cy - h_box / 2.0f;

                    bboxes[cls_id].push_back(cv::Rect2d(x, y, w_box, h_box));
                    scores[cls_id].push_back(score);
                }
            }
        }
    }

    for (int idx = 0; idx < 3; ++idx) {
    float* raw = output_data[idx];
    int h = feature_h[idx];
    int w = feature_w[idx];
    auto& anchors = *anchor_sets[idx];
    int stride = strides[idx];
    int total_expected = h * w * anchors.size() * (5 + classes_num_);
    // LOG("idx="<<idx<<" h="<<h<<" w="<<w<<" anchors="<<anchors<<" classes="<<class<<" total_expected="<<total_expected);
    printf("Layer %d: h=%d w=%d anchors=%zu classes=%d total_expected=%d\n",
           idx, h, w, anchors.size(), classes_num_, total_expected);
    // 可选：尝试获取实际模型输出的元素个数（取决于你的 DNN 接口）
    }
}

void YOLOv5Detector::applyNMS(std::vector<std::vector<cv::Rect2d>>& bboxes,
                              std::vector<std::vector<float>>& scores,
                              std::vector<cv::Rect2d>& final_boxes,
                              std::vector<float>& final_scores,
                              std::vector<int>& final_class_ids) {
    for (int cls = 0; cls < classes_num_; ++cls) {
        std::vector<int> indices;
        cv::dnn::NMSBoxes(bboxes[cls], scores[cls], score_threshold_,
                          nms_threshold_, indices, 1.0f, nms_top_k_);
        for (int idx : indices) {
            final_boxes.push_back(bboxes[cls][idx]);
            final_scores.push_back(scores[cls][idx]);
            final_class_ids.push_back(cls);
        }
    }
}

void YOLOv5Detector::infer(const cv::Mat& nv12_image, std::vector<float*>& output_data) {
    // 1. 准备输入tensor
    hbDNNTensor input;
    hbDNNTensorProperties input_prop;
    hbDNNGetInputTensorProperties(&input_prop, dnn_handle_, 0);
    input.properties = input_prop;
    int input_size = input_h_ * input_w_ * 3 / 2;
    hbSysAllocCachedMem(&input.sysMem[0], input_size);
    memcpy(input.sysMem[0].virAddr, nv12_image.data, input_size);
    hbSysFlushMem(&input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    
    // 2. 创建任务句柄
    hbDNNTaskHandle_t task_handle = nullptr;
    hbDNNInferCtrlParam ctrl_param;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&ctrl_param);
    hbDNNTensor* output = output_tensors_.data();
    
    // 3. 执行推理
    hbDNNInfer(&task_handle, &output, &input, dnn_handle_, &ctrl_param);
    hbDNNWaitTaskDone(task_handle, 0);
    hbDNNReleaseTask(task_handle);
    
    // 4. 刷新输出内存缓存
    for (auto& tensor : output_tensors_) {
        hbSysFlushMem(&tensor.sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
        output_data.push_back(static_cast<float*>(tensor.sysMem[0].virAddr));
    }
    
    hbSysFreeMem(&input.sysMem[0]);
}