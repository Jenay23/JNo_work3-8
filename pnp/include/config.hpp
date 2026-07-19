/// @file config.hpp
/// @brief 全局配置参数
/// @details 定义装甲板物理尺寸、相机参数、检测阈值等编译期常量
/// @author JNo
/// @date 2026-07-15

#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <opencv2/core.hpp>

/// @brief 装甲板物理宽度 (mm)
constexpr float ARMOR_WIDTH  = 135.0f;
/// @brief 装甲板物理高度 (mm)
constexpr float ARMOR_HEIGHT = 125.0f;

/// @brief 相机内参矩阵 3x3
extern cv::Mat CAMERA_MATRIX;
/// @brief 畸变系数向量 (k1, k2, p1, p2, k3)
extern cv::Mat DIST_COEFFS;

constexpr int ENEMY_COLOR_RED  = 0;  ///< 红色敌方颜色标识
constexpr int ENEMY_COLOR_BLUE = 1;  ///< 蓝色敌方颜色标识
/// @brief 当前敌方颜色（按实际比赛修改）
constexpr int ENEMY_COLOR = ENEMY_COLOR_BLUE;

/// @brief 灯条最小轮廓面积 (像素)，过滤小噪点
constexpr double LIGHT_MIN_AREA = 80.0;
/// @brief 灯条宽高比上限，灯条应细长
constexpr double LIGHT_MAX_RATIO = 0.4;
/// @brief 灯条最小凸度 = 轮廓面积 / 外接矩形面积
constexpr double LIGHT_CONTOUR_MIN_SOLIDITY = 0.5;

/// @brief 细长尾巴宽度阈值 (像素)
constexpr int TAIL_MAX_WIDTH = 6;
/// @brief 细长尾巴面积阈值 (像素)
constexpr double TAIL_MAX_AREA = 300.0;

#endif
