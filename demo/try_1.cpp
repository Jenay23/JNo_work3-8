#include "stdio.h"
#include<iostream> 
#include <opencv2/opencv.hpp>
#include <float.h>
using namespace std;
using namespace cv;

// 灯条类
class LightDescriptor
{
public:
    float width, length, angle, area;
    cv::Point2f center;
    cv::Point2f tl, tr, bl, br;
public:
    LightDescriptor() {};
    LightDescriptor(const cv::RotatedRect& light, const vector<cv::Point>& contour)
    {
        width = light.size.width;
        length = light.size.height;
        center = light.center;
        angle = light.angle;
        area = light.size.area();

        cv::Point2f pts[4];
        light.points(pts);

        cv::Point2f corners[4];
        for (int k = 0; k < 4; k++) {
            float min_d = FLT_MAX;
            int best = 0;
            for (int m = 0; m < (int)contour.size(); m++) {
                float d = norm(pts[k] - cv::Point2f(contour[m]));
                if (d < min_d) { min_d = d; best = m; }
            }
            corners[k] = contour[best];
        }

        for (int k = 0; k < 3; k++)
            for (int m = k + 1; m < 4; m++)
                if (corners[k].y > corners[m].y) swap(corners[k], corners[m]);

        if (corners[0].x > corners[1].x) swap(corners[0], corners[1]);
        if (corners[2].x > corners[3].x) swap(corners[2], corners[3]);

        tl = corners[0]; tr = corners[1];
        bl = corners[2]; br = corners[3];
    }
};

// ===================== 蓝色通道分离版预处理（已修正顺序） =====================
Mat preprocess_blue(const Mat& frame) {
    // 1. 先高斯模糊降噪（必须在二值化之前）
    Mat blurred;
    GaussianBlur(frame, blurred, Size(5, 5), 0);

    // 2. 通道分离，取蓝色通道 channels[0]
    Mat channels[3];
    split(blurred, channels);

    // 3. 二值化提取亮蓝色区域
    Mat binary;
    threshold(channels[0], binary, 200, 255, THRESH_BINARY);

    // 4. 形态学优化：去噪+连接断裂
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(binary, binary, MORPH_OPEN, kernel);
    morphologyEx(binary, binary, MORPH_CLOSE, kernel);

    return binary;
}

int main()
{
    // ========== 核心改动1：换成读取单张图片，替换成你的图片路径 ==========
    Mat frame = imread("/home/jno/Desktop/BKD2026培训/opencv/demo/_1.jpeg");
    if (frame.empty()) {
        cout << "图片读取失败，请检查路径" << endl;
        return -1;
    }

    // 预处理
    Mat binary = preprocess_blue(frame);

    // 轮廓检测
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binary, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 筛选灯条
    vector<LightDescriptor> lightInfos;
    for (int i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        // 修复：fitEllipse 要求至少5个点
        if (area < 5 || contours[i].size() < 5)
            continue;

        RotatedRect Light_Rec = fitEllipse(contours[i]);
        
        // 修复：先统一宽高，再判断长宽比
        float w = Light_Rec.size.width;
        float h = Light_Rec.size.height;
        if (w > h) swap(w, h);
        if (h / w > 10 || h / w < 2)
            continue;
            
        lightInfos.push_back(LightDescriptor(Light_Rec, contours[i]));
    }

    // 灯条配对 + 绘制
    for (size_t i = 0; i < lightInfos.size(); i++) {
        for (size_t j = i + 1; j < lightInfos.size(); j++) {
            LightDescriptor *left, *right;
            if (lightInfos[i].center.x < lightInfos[j].center.x) {
                left  = &lightInfos[i];
                right = &lightInfos[j];
            } else {
                left  = &lightInfos[j];
                right = &lightInfos[i];
            }

            float angleGap = abs(left->angle - right->angle);
            float meanAngle = (left->angle + right->angle) / 2;
            bool angleSym = (left->angle - meanAngle) * (right->angle - meanAngle) < 0;
            float meanLen = (left->length + right->length) / 2;
            float yGap = abs(left->center.y - right->center.y);
            float yGap_ratio = yGap / meanLen;
            float xGap = abs(left->center.x - right->center.x);
            float ratio = xGap / meanLen;

            if ((angleGap > 35 || (angleGap > 15 && !angleSym)) ||
                yGap_ratio > 0.25 ||
                ratio < 1.2 ||
                ratio > 3.5) {
                continue;
            }

            line(frame, left->tl,  right->tr, Scalar(0, 0, 255), 2);
            line(frame, right->tr, right->br, Scalar(0, 0, 255), 2);
            line(frame, right->br, left->bl,  Scalar(0, 0, 255), 2);
            line(frame, left->bl,  left->tl,  Scalar(0, 0, 255), 2);
        }
    }

    // 显示结果
    namedWindow("二值掩膜", WINDOW_FREERATIO);
    namedWindow("识别结果", WINDOW_FREERATIO);
    imshow("二值掩膜", binary);
    imshow("识别结果", frame);

    // ========== 核心改动2：按任意键才关闭窗口，方便你慢慢看效果 ==========
    waitKey(0);
    destroyAllWindows();

    return 0;
}
