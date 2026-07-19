/// @file armor_detector_base.hpp
/// @brief 装甲板检测基类
/// @details 定义检测器接口，提供细长尾巴移除和灯条拟合公共方法
/// @author JNo
/// @date 2026-07-15

#ifndef ARMOR_DETECTOR_BASE_HPP
#define ARMOR_DETECTOR_BASE_HPP

#include <opencv2/core.hpp>
#include <vector>

class ArmorDetectorBase
{
public:
    virtual ~ArmorDetectorBase() = default;

    virtual cv::Mat preprocess(const cv::Mat& frame) = 0;

    void removeThinTails(cv::Mat& binary, int maxWidth, double maxArea);

    std::vector<cv::RotatedRect> findLightsFromBinary(const cv::Mat& binary);
};

#endif
