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
    std::unique_lock<std::mutex> lock(mtx_);
    inference_queue_.push({nv12_image.clone(), image_id});
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
        postprocess_queue_.push({task.image, task.id});
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
            task = postprocess_queue_.front();
            postprocess_queue_.pop();
        }
        
        // 执行后处理
        Result res;
        // 1. 从output_tensors_中读取数据
        // 2. 解码边界框
        // 3. 执行NMS (具体实现可参考原代码)
        // 这里为占位示例
        // res.bboxes = ...;
        
        std::unique_lock<std::mutex> lock(mtx_);
        results_[task.id] = res;
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