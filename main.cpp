#include <iostream>
#include <vector>
#include <cmath>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "GPIOlib.h"

#define _DEBUG

using namespace cv;
using namespace GPIO;

const std::string CAM_PATH = "/dev/video0";
const int CANNY_LOWER_BOUND = 100;
const int CANNY_UPPER_BOUND = 200;
const int MY_RHO = 2;
const double THETA = CV_PI / 90.0;
const int HOUGH_LINE_THRESHOLD = 50;
const int MIN_LINE_LENGTH = 60;
const int MAX_LINE_GAP = 50;
const double BOTTOM_DISTANCE_THRESHOLD = 50;
const double BISECTOR_RATIO_THRESHOLD = 1.5590321636451379;
const int SPEED = 50;
const int TURN_ANGLE = 5;

enum MoveState {
    move_forward,
    move_left,
    move_right
};

double ratio(Vec4i line);

double ratio(double x1, double y1, double x2, double y2);

double dot(std::vector<double> vector1, std::vector<double> vector2);

Vec4i findClosestLine(std::vector<Vec4i> *lines, double kernel_x, double kernel_y);

Vec4i findAngleBisector(const Vec4i &leftLine, const Vec4i &rightLine);

MoveState generateNextMoveState(const Vec4i &leftLine, const Vec4i &rightLine, const Vec4i &bisector,
                                const Vec4i &midLine);

inline double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

inline bool emptyLine(Vec4i vec) {
    return vec[0] == 0 && vec[1] == 0 && vec[2] == 0 && vec[3] == 0;
}

int main() {
    // 从摄像头获取图片信息
//    VideoCapture capture(CAM_PATH);
//    if (!capture.isOpened())
//    {
//        capture.open(atoi(CAM_PATH.c_str()));
//    }
//
//    double width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
//    double height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
//
//    std::clog << "Frame Size: " << width << height << std::endl;

    Mat image = imread("/Users/jundaliao/CLionProjects/NaiveCar/examples/example5.jpg", IMREAD_COLOR);
    Mat gray_image, contour;

    double height = image.rows;
    double width = image.cols;

    // Init the libGPIO
#ifdef _ON_RASPBERRY
    init();
#endif

    while (true) {
//        capture >> image;
        if (image.empty())
            break;

        cvtColor(image, gray_image, COLOR_RGB2GRAY);
        Canny(gray_image, contour, CANNY_LOWER_BOUND, CANNY_UPPER_BOUND);

        std::vector<Vec4i> lines, leftLines, rightLines;
        HoughLinesP(contour, lines, MY_RHO, THETA, HOUGH_LINE_THRESHOLD, MIN_LINE_LENGTH, MAX_LINE_GAP);

        // take part into different types and filter useless lines
        for (Vec4i &vec: lines) {

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
                else if (left) {
                    if (-ratio > 0.2)
                        leftLines.push_back(vec);
                } else {
                    if (ratio > 0.2)
                        rightLines.push_back(vec);
                }
            } else {
                if (left)
                    leftLines.push_back(vec);
                else
                    rightLines.push_back(vec);
            }

        }

        if (leftLines.size() + rightLines.size() == 0) { // 没有检测到直线，有可能是出界了或者在正中央，应当向前行进，或者重复上一动作
            std::clog << "没有检测到直线，应该向正前方向行进或重复上一动作" << std::endl;
        }

        double kernel_x = width / 2, kernel_y = height;

        Vec4i leftLine = findClosestLine(&leftLines, kernel_x, kernel_y);
        Vec4i rightLine = findClosestLine(&rightLines, kernel_x, kernel_y);

        Vec4i midLine = Vec4i(int(kernel_x), 0, int(kernel_x), int(kernel_y));
        Vec4i bisector = findAngleBisector(leftLine, rightLine);

#ifdef _ON_RASPBERRY
        MoveState nextAction = generateNextMoveState(leftLine, rightLine, bisector, midLine);

        controlLeft(FORWARD, SPEED);
        controlRight(FORWARD, SPEED);

        if (nextAction == move_left) {
            turnTo(-TURN_ANGLE);
        } else if (nextAction == move_right) {
            turnTo(TURN_ANGLE);
        }

        delay(1000);
#endif


#ifdef _DEBUG

        std::cout << leftLine << " " << rightLine << " " << midLine << std::endl;
        std::vector<Vec4i> test;
        test.push_back(leftLine);
        test.push_back(rightLine);
        test.push_back(midLine);

        for (Vec4i vec: test) { // just show
            int x, y;
            x = vec[0] - vec[2];
            y = vec[1] - vec[3];

            line(image, Point(vec[0] + 10 * x, vec[1] + 10 * y), Point(vec[2] - 10 * x, vec[3] - 10 * y),
                 Scalar(255, 0, 0), 5);
        }

        std::cout << ratio(leftLine) << " " << ratio(rightLine) << std::endl;
//        std::cout << "The car will " << nextAction << std::endl;

        imshow("show lines", image);
        waitKey(0);


        break;

#endif
    }

}

double ratio(Vec4i line) {
    return ratio(line[0], line[1], line[2], line[3]);
}

double ratio(double x1, double y1, double x2, double y2) {
    if (x1 == x2)
        return INFINITY;
    else
        return (y2 - y1) / (x2 - x1);
}

double dot(std::vector<double> vector1, std::vector<double> vector2) {
    double sum = 0.0;
    for (int i = 0; i < vector1.size(); i++)
        sum += vector1[i] * vector2[i];
    return sum;
}

Vec4i findClosestLine(std::vector<Vec4i> *lines, double kernel_x, double kernel_y) {
    double dist = INFINITY;
    Vec4i vec4i;
    for (Vec4i vec: *lines) {
        int x1 = vec[0], y1 = vec[1], x2 = vec[2], y2 = vec[3];
        double lineLength = distance(x1, y1, x2, y2);

        std::vector<double> vec1, vec2;
        vec1.push_back(x1 - kernel_x);
        vec1.push_back(y1 - kernel_y);
        vec2.push_back(x2 - kernel_x);
        vec2.push_back(y2 - kernel_y);

        double lenVec1 = sqrt(dot(vec1, vec1));
        double lenVec2 = sqrt(dot(vec2, vec2));

        double cos = dot(vec1, vec2) * 1.0 / lenVec1 / lenVec2;
        double sin = sqrt(1 - cos * cos);

        double currentDist = lenVec1 * lenVec2 * sin / lineLength;

        if (dist > currentDist) {
            dist = currentDist;
            vec4i = vec;
        }

    }

    return vec4i;
}


MoveState generateNextMoveState(const Vec4i &leftLine, const Vec4i &rightLine, const Vec4i &bisector,
                                const Vec4i &midLine) {
    bool left = emptyLine(leftLine), right = emptyLine(rightLine);
    if (left && right)
        return move_forward;
    if (right)
        return move_right;
    if (left)
        return move_left;

    double leftRatio = ratio(leftLine), rightRatio = ratio(rightLine);
    if (leftRatio != INFINITY && rightRatio != INFINITY) {
        // Calculate distance between bottom intersection
        double midLineBottomX = midLine[0];

        double bisectorRatio = ratio(bisector);
        double height = abs(midLine[3] - midLine[1]);
        double bisectorBottomX = (bisector[1] - height) / bisectorRatio + bisector[0];

        double bottomDistance = midLineBottomX - bisectorBottomX;

        if (bottomDistance > 0 && bottomDistance < BOTTOM_DISTANCE_THRESHOLD
            && bisectorRatio < 0 && -bisectorRatio < BISECTOR_RATIO_THRESHOLD) {
            // If it heads northwest and is at the right of the path
            return move_right;
        } else if (bottomDistance > 0 && bisectorRatio > 0) {
            // Head northeast and is at the right of the path
            return move_left;
        } else if (bottomDistance < 0 && -bottomDistance < BOTTOM_DISTANCE_THRESHOLD
                   && bisectorRatio > 0 && bisectorRatio < BISECTOR_RATIO_THRESHOLD) {
            // Head northeast and is at the left of the path
            return move_left;
        } else if (bottomDistance < 0 && bisectorRatio < 0) {
            // Head northeast and is at the right of the path
            return move_right;
        }
    }

    return move_forward;
}

Vec4i findAngleBisector(const Vec4i &leftLine, const Vec4i &rightLine) {
    return cv::Vec4i();
}
