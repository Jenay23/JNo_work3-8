#include "stdio.h"
#include<iostream> 
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;

// ============================================================
// 灯条类：存储灯条的关键信息，用于后续匹配
// ============================================================
class LightDescriptor
{
public:
    float width, length, angle, area;   // 灯条的宽、长、角度、面积
    cv::Point2f center;                 // 灯条中心点坐标
public:
    LightDescriptor() {};
    // 用旋转矩形 RotatedRect 构造灯条描述符
    LightDescriptor(const cv::RotatedRect& light)
    {
        width  = light.size.width;
        length = light.size.height;
        center = light.center;
        angle  = light.angle;
        area   = light.size.area();
    }
};

int main()
{
    // ============================================================
    // 1. 读取单张图片（替换为你的图片路径）
    // ============================================================
    Mat frame = imread("/home/jno/Desktop/BKD2026培训/opencv/demo/_1.jpeg");
    if (frame.empty()) {
        cout << "图片读取失败，请检查路径" << endl;
        return -1;
    }

    // ============================================================
    // 2. 图像预处理：提取蓝色通道 + 二值化
    // ============================================================
    Mat channels[3], binary;
    split(frame, channels);             // 分离 BGR 通道，channels[0] = 蓝色通道
    threshold(channels[0], binary, 220, 255, THRESH_BINARY);  // 蓝色通道二值化

    // ============================================================
    // 3. 形态学处理：去除噪点，连通断开的区域
    // ============================================================
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(binary, binary, MORPH_OPEN, kernel);   // 开运算：去白噪点
    morphologyEx(binary, binary, MORPH_CLOSE, kernel);  // 闭运算：填充小空洞

    // ============================================================
    // 4. 轮廓检测
    // ============================================================
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binary, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // ============================================================
    // 5. 筛选灯条：按面积、点数、长宽比过滤
    // ============================================================
    vector<LightDescriptor> lightInfos;
    for (int i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        // minAreaRect 对轮廓点数有要求，至少 5 个点
        if (area < 5 || contours[i].size() < 5)
            continue;

        RotatedRect Light_Rec = minAreaRect(contours[i]);  // 最小外接旋转矩形

        // 统一宽高，确保 h >= w，再判断长宽比
        float w = Light_Rec.size.width;
        float h = Light_Rec.size.height;
        if (w > h) swap(w, h);
        // 灯条应细长：长宽比在 2~10 之间
        if (h / w > 10 || h / w < 2)
            continue;

        lightInfos.push_back(LightDescriptor(Light_Rec));
    }

    // ============================================================
    // 6. 灯条配对 + 由两个灯条中心构造装甲板矩形
    // ============================================================
    for (size_t i = 0; i < lightInfos.size(); i++) {
        for (size_t j = i + 1; j < lightInfos.size(); j++) {
            LightDescriptor& leftLight  = lightInfos[i];
            LightDescriptor& rightLight = lightInfos[j];

            // 角度差
            float angleGap = abs(leftLight.angle - rightLight.angle);
            // 角度均值，用于判断两个灯条是否在均值两侧对称倾斜
            float meanAngle = (leftLight.angle + rightLight.angle) / 2;
            // 角度对称性：歪视角时两个灯条会呈"八"字型张开，符号相反
            bool angleSym = (leftLight.angle - meanAngle) * (rightLight.angle - meanAngle) < 0;
            // 两个灯条的平均长度
            float meanLen  = (leftLight.length + rightLight.length) / 2;
            // Y 方向差距（应尽量接近，说明在同一水平线上）
            float yGap     = abs(leftLight.center.y - rightLight.center.y);
            float yGap_ratio = yGap / meanLen;
            // X 方向差距（应有一定宽度）
            float xGap     = abs(leftLight.center.x - rightLight.center.x);
            float ratio    = xGap / meanLen;

            // 匹配条件：角度差放宽到35°，同时要求对称（应对歪视角）
            if ((angleGap > 35 || (angleGap > 15 && !angleSym)) ||
                yGap_ratio > 0.25 ||
                ratio < 1.2 ||
                ratio > 3.5) {
                continue;
            }

            // 计算装甲板中心点：取两个椭圆中心（灯条中心）的中点
            Point2f center = Point2f(
                (leftLight.center.x + rightLight.center.x) / 2.0f,
                (leftLight.center.y + rightLight.center.y) / 2.0f
            );

            // 以两个椭圆中心连线方向作为矩形的长边方向
            Point2f dir = Point2f(
                rightLight.center.x - leftLight.center.x,
                rightLight.center.y - leftLight.center.y
            );
            float dist = cv::norm(dir);
            if (dist < 1e-5f) {
                continue;
            }
            dir *= 1.0f / dist;

            // 垂直于长边方向的向量，用来构造矩形的短边方向
            Point2f perp(-dir.y, dir.x);

            // 以两个椭圆中心之间的距离作为矩形宽度，两个灯条平均长度作为矩形高度
            float halfW = xGap / 2.0f;
            float halfH = meanLen / 2.0f;

            Point2f vertices[4] = {
                center + halfW * dir + halfH * perp,
                center - halfW * dir + halfH * perp,
                center - halfW * dir - halfH * perp,
                center + halfW * dir - halfH * perp
            };

            vector<Point> rectPoints;
            for (int k = 0; k < 4; k++) {
                rectPoints.push_back(Point(cvRound(vertices[k].x), cvRound(vertices[k].y)));
            }
            polylines(frame, rectPoints, true, Scalar(0, 0, 255), 2);
            circle(frame, Point(cvRound(center.x), cvRound(center.y)), 3, Scalar(255, 0, 0), -1);
        }
    }

    // ============================================================
    // 7. 显示结果
    // ============================================================
    namedWindow("二值掩膜", WINDOW_FREERATIO);
    namedWindow("识别结果", WINDOW_FREERATIO);
    imshow("二值掩膜", binary);
    imshow("识别结果", frame);

    waitKey(0);          // 按任意键关闭
    destroyAllWindows();

    return 0;
}
