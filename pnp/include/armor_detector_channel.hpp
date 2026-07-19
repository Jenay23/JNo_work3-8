/// @file armor_detector_channel.hpp
/// @brief 装甲板检测器（通道分离版）
/// @details 通过 BGR 通道差分提取灯条核心区域
/// @author JNo
/// @date 2026-07-15

#ifndef ARMOR_DETECTOR_CHANNEL_HPP
#define ARMOR_DETECTOR_CHANNEL_HPP

#include <opencv2/core.hpp>
#include <vector>
#include "armor_detector_base.hpp"

/// @brief 装甲板检测器（通道分离版）
/// @details 通过 BGR 通道差分提取灯条核心区域
class ArmorDetectorChannel : public ArmorDetectorBase
{
public:
    ArmorDetectorChannel();

    cv::Mat preprocess(const cv::Mat& frame) override;
};

#endif
