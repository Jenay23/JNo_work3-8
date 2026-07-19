/// @file armor_detector.hpp
/// @brief 装甲板检测模块
/// @details 提供装甲板颜色预处理、灯条拟合等功能，
///          输出灯条的旋转矩形供后续配对和 PnP 解算
/// @author JNo
/// @date 2026-07-15

#ifndef ARMOR_DETECTOR_HPP
#define ARMOR_DETECTOR_HPP

#include <opencv2/core.hpp>
#include <vector>
#include "armor_detector_base.hpp"

/// @brief 装甲板检测器（HSV 版）
/// @details 通过对 BGR 图像做 HSV 颜色预处理提取灯条核心区域
class ArmorDetector : public ArmorDetectorBase
{
public:
    ArmorDetector();

    cv::Mat preprocess(const cv::Mat& frame) override;
};

#endif // ARMOR_DETECTOR_HPP
