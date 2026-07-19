/// @file pose_solver.hpp
/// @brief 位姿解算模块
/// @details 基于 PnP 算法求解装甲板在相机坐标系下的位姿，
///          支持相机内参设置、EPnP+迭代法求解、欧拉角转换
/// @author JNo
/// @date 2026-07-15

#ifndef POSE_SOLVER_HPP
#define POSE_SOLVER_HPP

#include <opencv2/core.hpp>
#include <vector>

/// @brief 位姿解算器
/// @details 使用 EPnP + 迭代法求解装甲板在相机坐标系（X 右、Y 下、Z 前）下的位姿，
///          支持欧拉角（yaw/pitch/roll）输出
class PoseSolver
{
public:
    /// @brief 构造函数
    PoseSolver();

    /// @brief 初始化装甲板 3D 模型点
    /// @details 计算装甲板四个角点的世界坐标
    /// 世界坐标系与相机坐标系一致：X 右、Y 下、Z 前
    /// @return 成功返回 true
    bool Init();

    /// @brief 设置相机内参和畸变系数
    /// @param camera_matrix 相机内参矩阵 3x3
    /// @param dist_coeffs 畸变系数向量
    void setCameraParams(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs);

    /// @brief 根据 2D-3D 对应点求解装甲板在相机坐标系下的位姿
    /// @param corners_2d 检测到的四个角点（像素坐标）
    /// @param rvec 输出旋转向量
    /// @param tvec 输出平移向量
    /// @return true 求解成功
    bool solve(const std::vector<cv::Point2f>& corners_2d,
               cv::Mat& rvec, cv::Mat& tvec);

    /// @brief 将旋转向量和平移向量转为欧拉角和位置
    /// @param rvec 旋转向量
    /// @param tvec 平移向量
    /// @param x 输出装甲板中心在相机系 X 坐标 (mm)
    /// @param y 输出装甲板中心在相机系 Y 坐标 (mm)
    /// @param z 输出装甲板中心在相机系 Z 坐标 (mm)
    /// @param yaw 输出偏航角 (度)
    /// @param pitch 输出俯仰角 (度)
    /// @param roll 输出翻滚角 (度)
    void toEulerAngles(const cv::Mat& rvec, const cv::Mat& tvec,
                       double& x, double& y, double& z,
                       double& yaw, double& pitch, double& roll);

private:
    /// @brief 初始化装甲板四个 3D 模型点
    /// @details 世界坐标系与相机坐标系一致：X 右、Y 下、Z 前
    void initObjectPoints();

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    std::vector<cv::Point3f> object_points_;  // 装甲板 3D 模型点
};

#endif // POSE_SOLVER_HPP
