// stereo_system.cpp
#include "dh_robot_pkg/stereo_system.hpp"
#include <stdexcept>
#include <vector>

StereoSystem::StereoSystem(const std::string& yaml_file) {
    cv::FileStorage fs(yaml_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("无法打开YAML文件: " + yaml_file);
    }

    // 读取图像尺寸
    int w = static_cast<int>(fs["image_width"]);
    int h = static_cast<int>(fs["image_height"]);
    img_size_ = cv::Size(w, h);

    // 读取左右相机内参和畸变系数
    fs["camera_matrix_left"]  >> K1_;
    fs["dist_coeffs_left"]    >> D1_;
    fs["camera_matrix_right"] >> K2_;
    fs["dist_coeffs_right"]   >> D2_;
    fs["rotation_matrix_left_to_right"]  >> R_;
    fs["translation_vector_left_to_right"] >> T_;

    // 执行立体校正，获得校正后的旋转矩阵和投影矩阵
    cv::stereoRectify(K1_, D1_, K2_, D2_, img_size_, R_, T_,
                      R1_, R2_, P1_, P2_, Q_,
                      cv::CALIB_ZERO_DISPARITY, 0.0,
                      img_size_, nullptr, nullptr);

    // 可选：可预先计算校正映射（此处暂不实现，因为仅对点操作）
}

double StereoSystem::computeDistance(float u1, float v1, float u2, float v2) {
    // 1. 三角测量得到原始距离（与之前相同）
    std::vector<cv::Point2f> left_pt(1, cv::Point2f(u1, v1));
    std::vector<cv::Point2f> right_pt(1, cv::Point2f(u2, v2));

    std::vector<cv::Point2f> left_rect, right_rect;
    cv::undistortPoints(left_pt, left_rect, K1_, D1_, R1_, P1_);
    cv::undistortPoints(right_pt, right_rect, K2_, D2_, R2_, P2_);

    cv::Mat pts1(2, 1, CV_64F), pts2(2, 1, CV_64F);
    pts1.at<double>(0,0) = left_rect[0].x;
    pts1.at<double>(1,0) = left_rect[0].y;
    pts2.at<double>(0,0) = right_rect[0].x;
    pts2.at<double>(1,0) = right_rect[0].y;

    cv::Mat pts4D;
    cv::triangulatePoints(P1_, P2_, pts1, pts2, pts4D);

    double w = pts4D.at<double>(3, 0);
    if (std::abs(w) < 1e-6) return -1.0;
    double X = pts4D.at<double>(0,0) / w;
    double Y = pts4D.at<double>(1,0) / w;
    double Z = pts4D.at<double>(2,0) / w;

    // 有效性检查（Z>0 且 10米内）
    if (Z <= 0.0 || Z > 10000.0) {
        // 仍调用滤波，但传入无效值，让滤波返回历史中位数
        return filterDistance(-1.0);
    }

    double dist_mm = std::sqrt(X*X + Y*Y + Z*Z);
    double raw_dist = dist_mm / 1000.0;

    // 应用滤波
    return filterDistance(raw_dist);
}

void StereoSystem::setFilterParams(double max_rel_error, size_t max_buffer_size) {
    max_rel_error_ = max_rel_error;
    max_buffer_size_ = max_buffer_size;
    // 如果当前缓冲区超过新大小，裁剪
    while (buffer_.size() > max_buffer_size_) {
        buffer_.pop_front();
    }
}

double StereoSystem::filterDistance(double raw_dist) {
    // 无效输入直接返回 -1，不更新缓冲区
    if (raw_dist <= 0.0) {
        // 如果缓冲区有历史值，可以返回历史中位数，也可返回 -1（建议返回历史值更平滑）
        if (buffer_.empty()) return -1.0;
        // 返回当前缓冲区的中位数（保持输出连续）
        std::deque<double> sorted = buffer_;
        std::sort(sorted.begin(), sorted.end());
        return sorted[sorted.size() / 2];
    }

    // 缓冲区未满（或为空）：直接加入，并返回该值（或也可返回中位数，但此时数据少）
    if (buffer_.size() < 3) {
        buffer_.push_back(raw_dist);
        if (buffer_.size() > max_buffer_size_) buffer_.pop_front(); // 控制大小
        // 返回当前所有值的平均值或中位数？为简单，返回 raw_dist 本身（也可返回中位数）
        return raw_dist;
    }

    // 计算缓冲区中位数
    std::deque<double> sorted = buffer_;
    std::sort(sorted.begin(), sorted.end());
    double median = sorted[sorted.size() / 2];

    // 相对误差判断
    double rel_error = std::abs(raw_dist - median) / median;
    if (rel_error <= max_rel_error_) {
        // 接受新值，加入缓冲区
        buffer_.push_back(raw_dist);
        if (buffer_.size() > max_buffer_size_) buffer_.pop_front();
        // 返回当前缓冲区的中位数（平滑输出）
        sorted = buffer_;
        std::sort(sorted.begin(), sorted.end());
        return sorted[sorted.size() / 2];
    } else {
        // 拒绝突变，返回旧中位数
        return median;
    }
}

void StereoSystem::resetBuffer() {
    buffer_.clear();
}
