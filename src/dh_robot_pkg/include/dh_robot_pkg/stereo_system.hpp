// StereoSystem.h
#pragma once

#include <opencv2/opencv.hpp>
#include <deque>
#include <string>

class StereoSystem {
public:
    explicit StereoSystem(const std::string& yaml_file);

    /**
     * @brief 计算并返回滤波后的距离（米）。
     *        内部自动执行：三角测量 → 异常值剔除 → 滑动中位数滤波。
     * @param u1,v1 左图像点坐标（像素）
     * @param u2,v2 右图像点坐标（像素）
     * @return 滤波后的距离（米），无效时返回 -1.0
     */
    double computeDistance(float u1, float v1, float u2, float v2);

    /**
     * @brief 重置滤波缓冲区（例如切换目标时调用）
     */
    void resetBuffer();

    /**
     * @brief 设置滤波参数
     * @param max_rel_error 最大允许相对误差（相对于中位数），默认 0.25
     * @param max_buffer_size 缓冲区最大长度，建议 5~20，默认 10
     */
    void setFilterParams(double max_rel_error = 0.25, size_t max_buffer_size = 10);

private:
    // 标定参数
    cv::Size img_size_;
    cv::Mat K1_, D1_, K2_, D2_, R_, T_;
    cv::Mat R1_, R2_, P1_, P2_, Q_;   // 校正矩阵

    // 滤波相关
    std::deque<double> buffer_;
    double max_rel_error_ = 0.25;
    size_t max_buffer_size_ = 10;

    /**
     * @brief 核心滤波函数：对原始距离进行滑动中位数滤波
     * @param raw_dist 本次三角测量得到的原始距离（米），可能为 -1.0 表示无效
     * @return 滤波后的距离（米），若缓冲区为空则返回 -1.0
     */
    double filterDistance(double raw_dist);
};