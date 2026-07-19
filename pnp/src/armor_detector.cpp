/// @file armor_detector.cpp
/// @brief 装甲板检测模块实现
/// @details 实现颜色预处理、细长尾巴移除、二值图灯条拟合等
/// @author JNo
/// @date 2026-07-15

#include "armor_detector.hpp"
#include "config.hpp"

#include <opencv2/imgproc.hpp>

ArmorDetector::ArmorDetector() = default;

/// ============================================================
///  颜色提取（HSV 法）
/// ============================================================
cv::Mat ArmorDetector::preprocess(const cv::Mat& frame)
{
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> ch;
    cv::split(hsv, ch);

    cv::Mat mask;
    if (ENEMY_COLOR == ENEMY_COLOR_RED)
    {
        cv::Mat m1, m2;
        cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(10, 255, 255), m1);
        cv::inRange(hsv, cv::Scalar(170, 0, 0), cv::Scalar(180, 255, 255), m2);
        cv::bitwise_or(m1, m2, mask);
    }
    else
    {
        cv::inRange(hsv, cv::Scalar(90, 0, 0), cv::Scalar(140, 255, 255), mask);
    }

    /// 核心响应 = (255 - sat) * val / 255
    /// 低饱和(浅蓝→白) × 高亮 → 高响应(核心)
    /// 高饱和(纯蓝光晕)           → 低响应
    cv::Mat inv_sat;
    cv::subtract(cv::Scalar(255), ch[1], inv_sat);

    cv::Mat resp;
    cv::multiply(inv_sat, ch[2], resp, 1.0 / 255.0, CV_32F);

    cv::Mat result;
    resp.copyTo(result, mask);

    cv::Mat enhanced;
    cv::normalize(result, enhanced, 0, 255, cv::NORM_MINMAX);
    enhanced.convertTo(enhanced, CV_8U);

    return enhanced;
}

