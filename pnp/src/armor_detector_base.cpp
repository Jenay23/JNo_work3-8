/// @file armor_detector_base.cpp
/// @brief 装甲板检测基类实现
/// @details 提供细长尾巴移除、二值图灯条拟合等公共功能
/// @author JNo
/// @date 2026-07-15

#include "armor_detector_base.hpp"
#include "config.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>

void ArmorDetectorBase::removeThinTails(cv::Mat& binary, int maxWidth, double maxArea)
{
    if (binary.empty() || binary.type() != CV_8UC1) return;

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat tmp = binary.clone();
    cv::findContours(tmp, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& c : contours)
    {
        double area = cv::contourArea(c);
        if (area <= 0) continue;

        std::vector<cv::Point> approx;
        cv::approxPolyDP(c, approx, 2.0, true);

        cv::RotatedRect r = cv::minAreaRect(approx);
        double w = r.size.width;
        double h = r.size.height;
        double shortSide = std::min(w, h);

        if (shortSide < static_cast<double>(maxWidth) && area < maxArea)
        {
            cv::drawContours(binary, std::vector<std::vector<cv::Point>>{c}, -1, cv::Scalar(0), cv::FILLED);
        }
    }
}

std::vector<cv::RotatedRect> ArmorDetectorBase::findLightsFromBinary(const cv::Mat& binary)
{
    std::vector<cv::RotatedRect> lights;

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, contours, hierarchy,
                     cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours)
    {
        double area = cv::contourArea(contour);
        if (area < LIGHT_MIN_AREA) continue;

        std::vector<cv::Point> hull;
        cv::convexHull(contour, hull);

        std::vector<cv::Point> approx;
        cv::approxPolyDP(hull, approx, 3.0, true);

        cv::RotatedRect rect = cv::minAreaRect(approx);

        {
            float& w = rect.size.width;
            float& h = rect.size.height;
            float& a = rect.angle;
            while (a >= 90.0f) a -= 180.0f;
            while (a < -90.0f) a += 180.0f;
            if (a >= 45.0f)
            {
                std::swap(w, h);
                a -= 90.0f;
            }
            else if (a < -45.0f)
            {
                std::swap(w, h);
                a += 90.0f;
            }
        }

        float ratio = rect.size.width / rect.size.height;
        if (ratio > LIGHT_MAX_RATIO) continue;

        double rect_area = rect.size.width * rect.size.height;
        double solidity = area / rect_area;
        if (solidity < LIGHT_CONTOUR_MIN_SOLIDITY) continue;

        lights.push_back(rect);
    }

    return lights;
}
