#include <string>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// A4纸的尺寸，单位为毫米
const float A4_WIDTH = 210.0f;
const float A4_HEIGHT = 297.0f;

// 相机内参
Mat cameraMatrix = (Mat_<double>(3, 3) << 628.4936068390919, 0, 336.4774598492871,
                                          0, 634.577462248767, 247.422085082431,
                                          0, 0, 1);
// 相机外参
Mat distCoeff = (Mat_<double>(1, 4) << 0.08586636536342734, -0.7408114607936157, 0.004639149773896727, -0.003628736274083093, 1.467420464418536);

// 定义一个A4纸对象
struct A4Paper {
    vector<Point2f> corners; // 四个角点
    Mat rVec; // 旋转向量
    Mat tVec; // 平移向量
};

VideoCapture cap1;
Mat mapx, mapy;
Size imageSize = Size(640, 480);

// 初始化函数
void img_init(void);

// 识别 A4 纸
A4Paper detectA4(Mat &im, int hmin, int smin, int vmin, int hmax, int smax, int vmax);

// 显示结果
void display(Mat &im, A4Paper &a4Paper);


void img_init(void) {
    // 初始化摄像头
    cap1.open(0);
    if (!cap1.isOpened()) {
        cerr << "Error: Cannot open camera." << endl;
        return;
    }
 
    cap1.set(CAP_PROP_FRAME_WIDTH, 640);
    cap1.set(CAP_PROP_FRAME_HEIGHT, 480);
}

// 识别 A4 纸
A4Paper detectA4(Mat &im, int hmin, int smin, int vmin, int hmax, int smax, int vmax) {
    A4Paper a4Paper;

    // 1. 色彩特征提取
    Mat hsv;
    cvtColor(im, hsv, COLOR_BGR2HSV);
    // 设置 A4 纸颜色范围
    Scalar lower_bound(hmin, smin, vmin);
    Scalar upper_bound(hmax, smax, vmax);
    Mat mask;
    inRange(hsv, lower_bound, upper_bound, mask);
    // 形态学操作
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(mask, mask, MORPH_OPEN, kernel);
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);

    // 2. 形状特征提取
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    for (auto &contour : contours) {
        // 近似为多边形
        vector<Point> approx;
        approxPolyDP(contour, approx, arcLength(contour, true) * 0.02, true);
        // 筛选矩形形状
        if (approx.size() == 4 && isContourConvex(approx)) {
            // 计算面积
            double area = contourArea(approx);
            // 筛选面积最大的轮廓
            if (area > 10000) {
                // 获取最小外接矩形
                Rect rect = boundingRect(approx);
                // 计算长宽比
                float ratio = (float)rect.width / rect.height;
                // 判断是否符合 A4 纸比例
                if (ratio > 0.7 && ratio < 1.5) {
                    a4Paper.corners = vector<Point2f>{
                        Point2f(rect.x, rect.y),
                        Point2f(rect.x + rect.width, rect.y),
                        Point2f(rect.x + rect.width, rect.y + rect.height),
                        Point2f(rect.x, rect.y + rect.height)
                    };
                    break;
                }
            }
        }
    }

    // 3. 计算 A4 纸的姿态
    if (!a4Paper.corners.empty()) {
        vector<Point3f> obj = vector<Point3f>{
            cv::Point3f(0, 0, 0),  // tl
            cv::Point3f(A4_WIDTH, 0, 0),  // tr
            cv::Point3f(A4_WIDTH, A4_HEIGHT, 0),  // br
            cv::Point3f(0, A4_HEIGHT, 0)  // bl
        };
        solvePnP(obj, a4Paper.corners, cameraMatrix, distCoeff, a4Paper.rVec, a4Paper.tVec, false, SOLVEPNP_ITERATIVE);
    }

    return a4Paper;
}

// 显示结果
void display(Mat &im, A4Paper &a4Paper) {
    // 如果找到 A4 纸，绘制边框和显示姿态信息
    if (!a4Paper.corners.empty()) {
        for (int i = 0; i < 4; i++) {
            line(im, a4Paper.corners[i], a4Paper.corners[(i + 1) % 4], Scalar(0, 255, 0), 2);
        }
        cout << "rVec:\
" << a4Paper.rVec << endl;
        cout << "tVec:\
" << a4Paper.tVec << endl;
    }
}

int main() {
    initUndistortRectifyMap(cameraMatrix, distCoeff, Mat::eye(3, 3, CV_32F), cameraMatrix, imageSize, CV_32FC1, mapx, mapy);
    img_init();
    namedWindow("yuantu", WINDOW_AUTOSIZE);

    // 设置滑动条
    int hmin = 0, smin = 0, vmin = 50, hmax = 180, smax = 255, vmax = 255;
    namedWindow("TrackBars", WINDOW_AUTOSIZE);
    createTrackbar("Hmin", "TrackBars", &hmin, 180);
    createTrackbar("Hmax", "TrackBars", &hmax, 180);
    createTrackbar("Smin", "TrackBars", &smin, 255);
    createTrackbar("Smax", "TrackBars", &smax, 255);
    createTrackbar("Vmin", "TrackBars", &vmin, 255);
    createTrackbar("Vmax", "TrackBars", &vmax, 255);

    Mat im;
    while (waitKey(1) != 'q') {
        cap1 >> im;
        if (im.empty()) break;

        // 畸变矫正
        remap(im, im, mapx, mapy, INTER_LINEAR);

        // 识别 A4 纸
        A4Paper a4Paper = detectA4(im, hmin, smin, vmin, hmax, smax, vmax);

        // 显示识别结果
        display(im, a4Paper);
        imshow("原图", im);
        waitKey(30);
    }

    return 0;
}
