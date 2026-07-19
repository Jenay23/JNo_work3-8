/// @file armor_detector_channel.cpp
/// @brief 装甲板检测器（通道分离版）实现
/// @details 实现 BGR 通道差分预处理，提取灯条核心区域
/// @author JNo
/// @date 2026-07-15

#include "armor_detector_channel.hpp"
#include "config.hpp"

ArmorDetectorChannel::ArmorDetectorChannel() = default;

/// ============================================================
///  通道分离预处理
///  在 BGR 空间直接做通道差分，提取灯条核心区域
///
///  原理：
///    - 蓝色敌方：灯条区域 B >> R 且 B >> G，白热化中心 B≈R≈G≈255
///      → 用 B - min(R, G) 可同时突出蓝色区域并抑制白色过曝
///    - 红色敌方：灯条区域 R >> B 且 R >> G
///      → 用 R - max(B, G)
///
///  输出为单通道灰度图，响应高的区域对应灯条核心
/// ============================================================
cv::Mat ArmorDetectorChannel::preprocess(const cv::Mat& frame)
{
    std::vector<cv::Mat> ch;
    cv::split(frame, ch);
    // ch[0]=B, ch[1]=G, ch[2]=R

    cv::Mat diff;

    if (ENEMY_COLOR == ENEMY_COLOR_RED)
    {
        // R - max(B, G)
        cv::Mat bg;
        cv::max(ch[0], ch[1], bg);
        cv::subtract(ch[2], bg, diff);
    }
    else
    {
        // B - min(R, G)
        cv::Mat rg;
        cv::min(ch[1], ch[2], rg);
        cv::subtract(ch[0], rg, diff);
    }

    cv::Mat result;
    cv::normalize(diff, result, 0, 255, cv::NORM_MINMAX);
    result.convertTo(result, CV_8U);

    return result;
}

