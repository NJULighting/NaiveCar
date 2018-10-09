#include <iostream>
#include <vector>
#include <cmath>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#define _DEBUG

using namespace cv;

const std::string CAM_PATH = "/dev/video0";
const int CANNY_LOWER_BOUND = 100;
const int CANNY_UPPER_BOUND = 200;
const int RHO = 2;
const double THETA = CV_PI / 90.0;
const int HOUGH_LINE_THRESHOLD = 50;
const int MIN_LINE_LENGTH = 60;
const int MAX_LINE_GAP = 50;


double dot(const std::vector<double> * vector1, const std::vector<double> * vector2);
Vec4i findClosestLine(const std::vector<Vec4i> * lines, double kernel_x, double kernel_y);

inline double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

int main()
{
    VideoCapture capture(CAM_PATH);
    if (!capture.isOpened())
    {
        capture.open(atoi(CAM_PATH.c_str()));
    }

    double width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    double height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);

    std::clog << "Frame Size: " << width << height << std::endl;

    Mat image;
    Mat gray_image, contour;

    while(true) {
        capture >> image;
        if (image.empty())
            break;

        cvtColor(image, gray_image, COLOR_RGB2GRAY);
        Canny(gray_image, contour, CANNY_LOWER_BOUND, CANNY_UPPER_BOUND);

        std::vector<Vec4i> lines, leftLines, rightLines;
        HoughLinesP(contour, lines, RHO, THETA, HOUGH_LINE_THRESHOLD, MIN_LINE_LENGTH, MAX_LINE_GAP);

        // take part into different types and filter useless lines
        for (Vec4i & vec: lines) {

            int x1 = vec[0], y1 = vec[1], x2 = vec[2], y2 = vec[3];
            bool left = false, right = false;

            if (x1 < width / 4 || x2 < width / 4)
                left = true;
            if (x1 > width * 3 / 4 || x2 > width * 3 / 4)
                right = true;
            if (!left && !right) // 贯通左右的线，我觉得他不太对
                continue;

            if (x1 != x2) {
                double ratio = (y1 - y2) * 1.0 / (x1 - x2);
                if (left && right) // 水平线，肯定不正常
                    continue;
                else if (left)
                    if (-ratio > 0.2)
                        leftLines.push_back(vec);
                else
                    if (ratio > 0.2)
                        rightLines.push_back(vec);

            } else {
                if (left)
                    leftLines.push_back(vec);
                else
                    rightLines.push_back(vec);
            }

        }

        if (leftLines.size() + rightLines.size() == 0) { // 没有检测到直线，有可能是出界了或者在正中央，应当向前行进
            std::clog << "没有检测到直线，应该向正前方向行进" << std::endl;
        }

        double kernel_x = width / 2, kernel_y = height;

        Vec4i leftLine = findClosestLine(&leftLines, kernel_x, kernel_y);
        Vec4i rightLine = findClosestLine(&rightLines, kernel_x, kernel_y);
    }

}

double dot(const std::vector<double> * vector1, const std::vector<double> * vector2)
{
    double sum = 0.0;
    for (int i = 0; i < vector1->size(); i++)
        sum += vector1[i] * vector2[i];
    return sum;
}

Vec4i findClosestLine(const std::vector<Vec4i> * lines, double kernel_x, double kernel_y)
{
    double dist = INFINITY;
    Vec4i vec4i;
    for (auto iter = lines->begin(); iter < lines->end(); iter++) {
        Vec4i vec = iter->conj();
        int x1 = vec[0], y1 = vec[1], x2 = vec[2], y2 = vec[3];
        double lineLength = distance(x1, y1, x2, y2);

        std::vector<double> vec1, vec2;
        vec1.push_back(x1 - kernel_x);
        vec1.push_back(y1 - kernel_y);
        vec2.push_back(x2 - kernel_x);
        vec2.push_back(y2 - kernel_y);

        double lenVec1 = sqrt(dot(&vec1, &vec1));
        double lenVec2 = sqrt(dot(&vec2, &vec2));

        double cos = dot(&vec1, &vec2) * 1.0 / lenVec1 / lenVec2;
        double sin = sqrt(1 - cos * cos);

        double currentDist = lenVec1 * lenVec2 * sin / lineLength;

        if (dist > currentDist) {
            dist = currentDist;
            vec4i = vec;
        }

    }

    return vec4i;

}