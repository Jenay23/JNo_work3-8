/// @file main.cpp
/// @brief 主程序入口，运行 HSV 和通道分离两种装甲板检测管线
/// @details 读取视频，同时运行两条检测管线并输出结果视频
/// @author JNo
/// @date 2026-07-15

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>

#include "armor_detector.hpp"
#include "armor_detector_channel.hpp"
#include "pose_solver.hpp"
#include "config.hpp"

cv::Mat CAMERA_MATRIX = (cv::Mat_<double>(3, 3) <<
    1789.2913422305087, 0.0, 702.93444417420312,
    0.0, 1789.214342198638, 557.65168675325629,
    0.0, 0.0, 1.0);

cv::Mat DIST_COEFFS = (cv::Mat_<double>(5, 1) <<
    -0.075551988063684988,
    0.13117079840026666,
    0.0014612753636792362,
    -0.001318090407814957,
    0.0);

/// @brief 在画面上绘制位姿信息
/// @param frame 原图（会被修改）
/// @param x, y, z 装甲板中心位置 (mm)
/// @param yaw, pitch, roll 欧拉角 (度)
void drawPoseInfo(cv::Mat& frame, double x, double y, double z,
                  double yaw, double pitch, double roll)
{
    int font = cv::FONT_HERSHEY_SIMPLEX;
    double scale = 0.6;
    int thickness = 2;
    cv::Scalar color(0, 255, 0);

    int margin_x = frame.cols - 320;
    int margin_y = 30;
    int line_h = 28;

    auto put = [&](const std::string& text, int offset)
    {
        cv::putText(frame, text,
                    cv::Point(margin_x, margin_y + offset * line_h),
                    font, scale, color, thickness);
    };

    put("Armor Pose:", 0);
    put(cv::format("X: %.1f mm", x), 1);
    put(cv::format("Y: %.1f mm", y), 2);
    put(cv::format("Z: %.1f mm", z), 3);
    put(cv::format("Dist: %.1f mm", std::sqrt(x*x + y*y + z*z)), 4);
    put(cv::format("Yaw: %.1f deg", yaw), 5);
    put(cv::format("Pitch: %.1f deg", pitch), 6);
    put(cv::format("Roll: %.1f deg", roll), 7);
}

/// @brief 灯条配对并进行 PnP 位姿解算
/// @param lights 检测到的灯条列表
/// @param solver 位姿解算器
/// @param result_viz 结果图（会被绘制）
void matchArmor(const std::vector<cv::RotatedRect>& lights,
                PoseSolver& solver, cv::Mat& result_viz)
{
    if (lights.size() < 2) return;

    cv::Point2f c1 = lights[0].center;
    cv::Point2f c2 = lights[1].center;
    cv::Point2f mid = (c1 + c2) * 0.5f;

    cv::circle(result_viz, c1, 4, cv::Scalar(255, 0, 0), -1);
    cv::circle(result_viz, c2, 4, cv::Scalar(255, 0, 255), -1);
    cv::line(result_viz, c1, c2, cv::Scalar(255, 255, 0), 1);
    cv::circle(result_viz, mid, 5, cv::Scalar(0, 0, 255), -1);
    cv::putText(result_viz, "mid", mid + cv::Point2f(5, -5),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);

    bool matched = false;
    for (size_t i = 0; i < lights.size() && !matched; ++i)
    {
        for (size_t j = i + 1; j < lights.size() && !matched; ++j)
        {
            float hi = lights[i].size.height;
            float hj = lights[j].size.height;

            c1 = lights[i].center;
            c2 = lights[j].center;

            cv::Point2f dir = c2 - c1;
            float dist = cv::norm(dir);
            dir *= 1.0f / dist;

            cv::Point2f perp(-dir.y, dir.x);

            float avg_h = (hi + hj) * 0.5f;
            float half_h = avg_h * 0.5f;

            std::vector<cv::Point2f> corners = {
                c1 + perp * half_h,
                c2 + perp * half_h,
                c2 - perp * half_h,
                c1 - perp * half_h,
            };

            for (const auto& pt : corners)
                cv::circle(result_viz, pt, 6, cv::Scalar(0, 255, 0), -1);
            for (int k = 0; k < 4; ++k)
                cv::line(result_viz, corners[k], corners[(k + 1) % 4],
                         cv::Scalar(0, 100, 0), 2);

            cv::Mat rvec, tvec;
            bool solved = solver.solve(corners, rvec, tvec);
            if (solved)
            {
                double x, y, z, yaw, pitch, roll;
                solver.toEulerAngles(rvec, tvec,
                                     x, y, z, yaw, pitch, roll);
                drawPoseInfo(result_viz, x, y, z, yaw, pitch, roll);
            }
            matched = true;
        }
    }

    if (!matched)
    {
        cv::putText(result_viz, "No armor plate detected",
                    cv::Point(30, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 0, 255), 2);
    }
}

/// @brief 单帧管线输出
struct PipelineResult
{
    cv::Mat preprocess_gray;  ///< 预处理灰度图
    cv::Mat binary_viz;       ///< 二值化 + 灯条可视化
    cv::Mat result_viz;       ///< 最终结果图（含绘制信息）
};

/// @brief 运行单帧检测管线
/// @param detector 检测器（可以是 HSV 或通道分离版）
/// @param solver 位姿解算器
/// @param frame 输入帧
/// @param frame_count 帧序号
/// @param low_val 二值化低阈值
/// @param high_val 二值化高阈值
/// @return PipelineResult 包含预处理、二值化和结果图
PipelineResult runPipeline(ArmorDetectorBase& detector, PoseSolver& solver,
                           const cv::Mat& frame, int frame_count,
                           int low_val, int high_val)
{
    PipelineResult res;
    res.result_viz = frame.clone();

    cv::Mat gray = detector.preprocess(frame);
    res.preprocess_gray = gray;

    cv::Mat binary;
    cv::inRange(gray, low_val, high_val, binary);

    cv::Mat morph = binary.clone();
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(morph, morph, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(morph, morph, cv::MORPH_OPEN, kernel);

    detector.removeThinTails(morph, TAIL_MAX_WIDTH, TAIL_MAX_AREA);

    std::vector<cv::RotatedRect> lights = detector.findLightsFromBinary(morph);

    std::sort(lights.begin(), lights.end(),
              [](const cv::RotatedRect& a, const cv::RotatedRect& b)
              {
                  return a.center.x < b.center.x;
              });

    cv::Mat morph_color;
    cv::cvtColor(morph, morph_color, cv::COLOR_GRAY2BGR);
    for (const auto& light : lights)
    {
        cv::Point2f pts[4];
        light.points(pts);
        for (int i = 0; i < 4; ++i)
        {
            cv::line(res.result_viz, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 0, 255), 2);
            cv::line(morph_color, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 0, 255), 2);
        }
    }
    res.binary_viz = morph_color;

    matchArmor(lights, solver, res.result_viz);

    cv::putText(res.result_viz, "Frame: " + std::to_string(frame_count),
                cv::Point(30, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 1);
    cv::putText(res.result_viz, "Lights: " + std::to_string(lights.size()),
                cv::Point(30, 55),
                cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 1);

    return res;
}

/// @brief 主函数
/// @param argc 参数个数
/// @param argv[1] 可选：视频路径，默认 ../data/raw.mp4
/// @return 0 成功，-1 失败
int main(int argc, char** argv)
{
    std::string video_path = (argc > 1) ? argv[1] : "../data/raw.mp4";
    cv::VideoCapture cap(video_path);
    if (!cap.isOpened())
    {
        std::cerr << "无法打开视频: " << video_path << std::endl;
        return -1;
    }

    double fps = cap.get(cv::CAP_PROP_FPS);
    int w = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int h = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    cv::Size frame_size(w, h);
    int total_frames = (int)cap.get(cv::CAP_PROP_FRAME_COUNT);

    std::cout << "视频已打开，分辨率: " << w << "x" << h
              << ", 总帧数: " << total_frames << ", FPS: " << fps << std::endl;

    ArmorDetector hsv_detector;
    ArmorDetectorChannel ch_detector;
    PoseSolver solver;
    solver.Init();
    solver.setCameraParams(CAMERA_MATRIX, DIST_COEFFS);

    int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

    cv::VideoWriter hsv_pre_w, hsv_bin_w, hsv_res_w;
    cv::VideoWriter ch_pre_w, ch_bin_w, ch_res_w;

    if (!hsv_pre_w.open("hsv_preprocess.avi", fourcc, fps, frame_size))
        { std::cerr << "无法创建 hsv_preprocess.avi" << std::endl; return -1; }
    if (!hsv_bin_w.open("hsv_binary.avi", fourcc, fps, frame_size))
        { std::cerr << "无法创建 hsv_binary.avi" << std::endl; return -1; }
    if (!hsv_res_w.open("hsv_result.avi", fourcc, fps, frame_size))
        { std::cerr << "无法创建 hsv_result.avi" << std::endl; return -1; }
    if (!ch_pre_w.open("ch_preprocess.avi", fourcc, fps, frame_size))
        { std::cerr << "无法创建 ch_preprocess.avi" << std::endl; return -1; }
    if (!ch_bin_w.open("ch_binary.avi", fourcc, fps, frame_size))
        { std::cerr << "无法创建 ch_binary.avi" << std::endl; return -1; }
    if (!ch_res_w.open("ch_result.avi", fourcc, fps, frame_size))
        { std::cerr << "无法创建 ch_result.avi" << std::endl; return -1; }

    std::cout << "将输出 6 个视频：" << std::endl;
    std::cout << "  HSV 方法: hsv_preprocess.avi, hsv_binary.avi, hsv_result.avi" << std::endl;
    std::cout << "  通道分离: ch_preprocess.avi, ch_binary.avi, ch_result.avi" << std::endl;

    cv::Mat frame;
    int frame_count = 0;

    while (cap.read(frame))
    {
        if (frame.empty()) continue;
        frame_count++;

        auto hsv_res = runPipeline(hsv_detector, solver, frame, frame_count, 30, 255);
        auto ch_res = runPipeline(ch_detector, solver, frame, frame_count, 100, 200);

        cv::Mat hsv_pre_bgr, ch_pre_bgr;
        cv::cvtColor(hsv_res.preprocess_gray, hsv_pre_bgr, cv::COLOR_GRAY2BGR);
        cv::cvtColor(ch_res.preprocess_gray, ch_pre_bgr, cv::COLOR_GRAY2BGR);

        hsv_pre_w.write(hsv_pre_bgr);
        hsv_bin_w.write(hsv_res.binary_viz);
        hsv_res_w.write(hsv_res.result_viz);

        ch_pre_w.write(ch_pre_bgr);
        ch_bin_w.write(ch_res.binary_viz);
        ch_res_w.write(ch_res.result_viz);

        cv::imshow("HSV Preprocess", hsv_pre_bgr);
        cv::imshow("HSV Binary", hsv_res.binary_viz);
        cv::imshow("HSV Result", hsv_res.result_viz);
        cv::imshow("CH Preprocess", ch_pre_bgr);
        cv::imshow("CH Binary", ch_res.binary_viz);
        cv::imshow("CH Result", ch_res.result_viz);

        if (cv::waitKey(30) == 27) break;
    }

    hsv_pre_w.release();
    hsv_bin_w.release();
    hsv_res_w.release();
    ch_pre_w.release();
    ch_bin_w.release();
    ch_res_w.release();

    cap.release();
    cv::destroyAllWindows();
    std::cout << "处理完成，共 " << frame_count << " 帧" << std::endl;
    return 0;
}
