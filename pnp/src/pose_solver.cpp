/// @file pose_solver.cpp
/// @brief 位姿解算模块实现
/// @details 实现 PnP 求解和欧拉角提取
/// @author JNo
/// @date 2026-07-15

#include "pose_solver.hpp"
#include "config.hpp"

#include <opencv2/calib3d.hpp>
#include <cmath>

PoseSolver::PoseSolver() = default;

bool PoseSolver::Init()
{
    initObjectPoints();
    return true;
}

void PoseSolver::setCameraParams(const cv::Mat& camera_matrix,
                                  const cv::Mat& dist_coeffs)
{
    camera_matrix_ = camera_matrix.clone();
    dist_coeffs_   = dist_coeffs.clone();
}

/// ============================================================
///  初始化装甲板的 3D 模型点
///
///  世界坐标系（与相机坐标系一致）：
///    原点 = 装甲板中心
///    X 向右（相机右侧）
///    Y 向下（相机下方）
///    Z 向前（指向被摄物体，即相机看向装甲板的方向）
///
///  四点位于装甲板平面 Z=0 上：
///    [0] = ( W/2, -H/2, 0)   装甲板左上
///    [1] = (-W/2, -H/2, 0)   装甲板右上
///    [2] = (-W/2,  H/2, 0)   装甲板右下
///    [3] = ( W/2,  H/2, 0)   装甲板左下
///
///  注："左上/右上"是装甲板自身视角
///      在相机看来，装甲板左侧 = 相机右侧 (+X)，装甲板上方 = 相机上方 (-Y)
/// ============================================================
void PoseSolver::initObjectPoints()
{
    object_points_.clear();
    object_points_.emplace_back( ARMOR_WIDTH / 2.0f, -ARMOR_HEIGHT / 2.0f, 0.0f);
    object_points_.emplace_back(-ARMOR_WIDTH / 2.0f, -ARMOR_HEIGHT / 2.0f, 0.0f);
    object_points_.emplace_back(-ARMOR_WIDTH / 2.0f,  ARMOR_HEIGHT / 2.0f, 0.0f);
    object_points_.emplace_back( ARMOR_WIDTH / 2.0f,  ARMOR_HEIGHT / 2.0f, 0.0f);
}

/// ============================================================
///  PnP 求解
///
///  步骤：
///    1. 先用 EPnP 快速估计一个初值（用 4 个点，EPnP 足够）
///    2. 再用迭代法以 EPnP 的结果为初值精化
///
///  输出：
///    rvec, tvec 直接表示装甲板在相机坐标系下的旋转和平移
///    即：P_cam = R * P_world + t，t 是装甲板中心在相机系下的位置
/// ============================================================
bool PoseSolver::solve(const std::vector<cv::Point2f>& corners_2d,
                        cv::Mat& rvec, cv::Mat& tvec)
{
    if (corners_2d.size() != 4) return false;

    /// 第 1 遍：EPnP（快速线性解，不需要初值）
    bool ok = cv::solvePnP(
        object_points_, corners_2d,
        camera_matrix_, dist_coeffs_,
        rvec, tvec,
        false,                          // 不需要初值
        cv::SOLVEPNP_EPNP
    );
    if (!ok) return false;

    /// 第 2 遍：迭代法精化（用 EPnP 的结果作为初值）
    ok = cv::solvePnP(
        object_points_, corners_2d,
        camera_matrix_, dist_coeffs_,
        rvec, tvec,
        true,                           // 用上面结果做初值
        cv::SOLVEPNP_ITERATIVE
    );
    if (!ok) return false;

    return true;
}

/// ============================================================
///  将 PnP 结果转为装甲板在相机坐标系下的位姿
///
///  输入：rvec, tvec（solvePnP 输出）
///        满足 P_cam = R * P_world + t
///        其中世界系与相机系同向，原点在装甲板中心
///
///  输出：
///    x, y, z    → 装甲板中心在相机坐标系下的位置 (mm)
///                 x: 向右为正
///                 y: 向下为正
///                 z: 向前为正（相机前方）
///    yaw        → 装甲板绕相机 Z 轴旋转（度）
///    pitch      → 装甲板绕相机 Y 轴旋转（度）
///    roll       → 装甲板绕相机 X 轴旋转（度）
///
///  注：solvePnP 返回的 t 即为装甲板中心在相机系下的位置，
///      R 即为装甲板→相机系的旋转矩阵，无需额外逆变换。
/// ============================================================
void PoseSolver::toEulerAngles(const cv::Mat& rvec, const cv::Mat& tvec,
                                double& x, double& y, double& z,
                                double& yaw, double& pitch, double& roll)
{
    /// 1. 旋转向量 → 旋转矩阵
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    /// 2. 平移向量直接输出 = 装甲板中心在相机系下的位置
    x = tvec.at<double>(0);
    y = tvec.at<double>(1);
    z = tvec.at<double>(2);

    /// 3. 从 R 提取欧拉角（ZYX 顺序）
    ///    R 即装甲板在相机系下的旋转
    double sy = std::sqrt(
        R.at<double>(0, 0) * R.at<double>(0, 0) +
        R.at<double>(1, 0) * R.at<double>(1, 0)
    );
    bool singular = sy < 1e-6;

    if (!singular)
    {
        yaw   = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
        pitch = std::atan2(-R.at<double>(2, 0), sy);
        roll  = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
    }
    else
    {
        yaw   = 0.0;
        pitch = std::atan2(-R.at<double>(2, 0), sy);
        roll  = std::atan2(R.at<double>(1, 2), R.at<double>(1, 1));
    }

    /// 4. 弧度 → 角度
    yaw   *= (180.0 / CV_PI);
    pitch *= (180.0 / CV_PI);
    roll  *= (180.0 / CV_PI);
}
