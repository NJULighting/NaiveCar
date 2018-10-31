//
// Created by 訾源 on 2018/10/18.
//

#include <iostream>
#include <vector>
#include <cmath>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "GPIOlib.h"
#include "PID.h"

#define _DEBUG
#define _ON_RASPBERRY

using namespace GPIO;
using namespace cv;

// constants about equipment
const std::string CAM_PATH = "/dev/video0";
const int CAM_HEIGHT = 480;
const int CAM_WIDTH = 640;

// constants about line recognization
const int CANNY_LOWER_BOUND = 100;
const int CANNY_UPPER_BOUND = 200;
const int MY_RHO = 2;
const double THETA = CV_PI / 90.0;
const int HOUGH_LINE_THRESHOLD = 50;
const int MIN_LINE_LENGTH = 60;
const int MAX_LINE_GAP = 50;

// constants abouts control
const int LEFT_SPEED = 50;
const int RIGHT_SPEED = 50;
const int LEFT_THRESHOLD = 40;
const int RIGHT_THRESHOLD = 40;
const int TURN_LEFT_ANGLE = 5;
const int TURN_RIGHT_ANGLE = 5;

// constants about PID control
const int PID_ANGLE_FACTOR = 5;

enum MoveState {
    move_forward,
    move_left,
    move_right
};

// inline methods
inline double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

inline bool emptyLine(const Vec4i &vec) {
    return vec[0] == 0 && vec[1] == 0 && vec[2] == 0 && vec[3] == 0;
}

// common method
double ratio(const Vec4i &line);

double ratio(double x1, double y1, double x2, double y2);

void lineFlip(Vec4i &line); // make the center of bottom (0, 0)
void findLeftAndRightLines(Mat &image, std::vector<Vec4i> &leftLines, std::vector<Vec4i> &rightLines);

Vec4i findClosestLine(const std::vector<Vec4i> &lines);

double dot(const std::vector<double> &vector1, const std::vector<double> &vector2);

void generalEquation(const Vec4i &line, double &A, double &B, double &C);

Vec4i findAngleBisector(const Vec4i &leftLine, const Vec4i &rightLine);

MoveState generateNextMoveState(const Vec4i &leftLine, const Vec4i &rightLine, const Vec4i &bisector);

void calculateCrossoverPoint(const Vec4i &line1, const Vec4i &line2, double &x, double &y);

void controlByNaiveMethod(MoveState state);

void controlByPid(PID &pid, Vec4i &leftLine, Vec4i &rightLine);

int main() {
    // init cam
    VideoCapture capture(CAM_PATH);
    if (!capture.isOpened()) {
        capture.open(atoi(CAM_PATH.c_str()));
    }

    Mat image;

    init();

    // Initialize PID controller
    PID pid;
    pid.Init(0.2, 0, 0.3);

    while (true) {
        if (!capture.isOpened())
            break;

        capture >> image;
        std::vector<Vec4i> leftLines, rightLines;
        findLeftAndRightLines(image, leftLines, rightLines);

        // find important lines
        Vec4i leftLine = findClosestLine(leftLines);
        Vec4i rightLine = findClosestLine(rightLines);
        Vec4i bisector = findAngleBisector(leftLine, rightLine);

        // get next moving state
        MoveState moveState = generateNextMoveState(leftLine, rightLine, bisector);
        controlByNaiveMethod(moveState);

//        controlByPid(pid, leftLine, rightLine);
    }

    return 0;
}


void lineFlip(Vec4i &line) {
    line[0] -= CAM_WIDTH / 2;
    line[2] -= CAM_WIDTH / 2;
    line[1] = -line[1] + CAM_HEIGHT;
    line[3] = -line[3] + CAM_HEIGHT;
}

void findLeftAndRightLines(Mat &image, std::vector<Vec4i> &leftLines, std::vector<Vec4i> &rightLines) {
    Mat gray_image, contour;
    cvtColor(image, gray_image, COLOR_RGB2GRAY);
    Canny(gray_image, contour, CANNY_LOWER_BOUND, CANNY_UPPER_BOUND);
    std::vector<Vec4i> lines;
    HoughLinesP(contour, lines, MY_RHO, THETA, HOUGH_LINE_THRESHOLD, MIN_LINE_LENGTH, MAX_LINE_GAP);

    for (Vec4i &vec: lines) {
        lineFlip(vec); // flip line

        int x1 = vec[0], y1 = vec[1], x2 = vec[2], y2 = vec[3];
        bool left = false, right = false;

        if (x1 < -CAM_WIDTH / 4 || x2 < -CAM_WIDTH / 4)
            left = true;
        if (x1 > CAM_WIDTH / 4 || x2 > CAM_WIDTH / 4)
            right = true;
        if ((!left && !right) || (left && right)) // 贯通左右的线或者在中间的短线
            continue;

        double _ratio = ratio(vec);
        if (left && (_ratio == INFINITY || _ratio > 0.2))
            leftLines.push_back(vec);
        if (right && (_ratio == INFINITY || _ratio < -0.2))
            rightLines.push_back(vec);
    }
}

double ratio(const Vec4i &line) {
    return ratio(line[0], line[1], line[2], line[3]);
}

double ratio(double x1, double y1, double x2, double y2) {
    if (x1 == x2)
        return INFINITY;
    else
        return (y2 - y1) / (x2 - x1);
}

Vec4i findClosestLine(const std::vector<Vec4i> &lines) {
    double dist = INFINITY;
    Vec4i closestLine;
    for (Vec4i line: lines) {
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
        double lineLength = distance(x1, y1, x2, y2);

        std::vector<double> vec1, vec2;
        vec1.push_back(x1 - 0.0);
        vec1.push_back(y1 - 0.0);
        vec2.push_back(x2 - 0.0);
        vec2.push_back(y2 - 0.0);

        double lenVec1 = sqrt(dot(vec1, vec1));
        double lenVec2 = sqrt(dot(vec2, vec2));

        double cos = dot(vec1, vec2) * 1.0 / lenVec1 / lenVec2;
        double sin = sqrt(1 - cos * cos);

        double currentDist = lenVec1 * lenVec2 * sin / lineLength;

        if (dist > currentDist) {
            dist = currentDist;
            closestLine = line;
        }
    }
    return closestLine;
}

double dot(const std::vector<double> &vector1, const std::vector<double> &vector2) {
    double sum = 0.0;
    for (int i = 0; i < vector1.size(); i++)
        sum += vector1[i] * vector2[i];
    return sum;
}

void generalEquation(const Vec4i &line, double &A, double &B, double &C) {
    A = line[3] - line[1];
    B = line[0] - line[2];
    C = line[2] * line[1] - line[0] * line[3];
}

void calculateCrossoverPoint(const Vec4i &line1, const Vec4i &line2, double &x, double &y) {
    double A1, B1, C1, A2, B2, C2;
    generalEquation(line1, A1, B1, C1);
    generalEquation(line2, A2, B2, C2);

    double m = A1 * B2 - A2 * B1;

    x = (C2 * B1 - C1 * B2) / m;
    y = (C1 * A2 - C2 * A1) / m;
}

Vec4i findAngleBisector(const Vec4i &leftLine, const Vec4i &rightLine) {
    bool leftEmpty = emptyLine(leftLine), rightEmpty = emptyLine(rightLine);
    if (leftEmpty || rightEmpty)
        return Vec4i(0, 0, 0, 0); // empty line

    // parallel?
    double leftRatio = ratio(leftLine), rightRatio = ratio(rightLine);
    if ((leftRatio == INFINITY && rightRatio == INFINITY) || leftRatio == rightRatio)
        return Vec4i(0, 0, 0, 0); // empty line


    double x1, y1, x2, y2;
    calculateCrossoverPoint(leftLine, rightLine, x1, y1);

    double l1, l2, v1x, v1y, v2x, v2y, v3x, v3y;
    v1x = leftLine[0] - leftLine[2];
    v2x = -(rightLine[0] - rightLine[2]);
    v1y = leftLine[1] - leftLine[3];
    v2y = -(rightLine[1] - rightLine[3]);
    l1 = sqrt(v1x * v1x + v1y * v1y);
    l2 = sqrt(v2x * v2x + v2y * v2y);

    v3x = v1x / l1 + v2x / l2;
    v3y = v1y / l1 + v2y / l2;

    x2 = x1 + v3x;
    y2 = y1 + v3y;

    return Vec4i(static_cast<int>(x1), static_cast<int>(y1), static_cast<int>(x2),
                 static_cast<int>(y2));
}

MoveState generateNextMoveState(const Vec4i &leftLine, const Vec4i &rightLine, const Vec4i &bisector) {
    bool leftEmpty = emptyLine(leftLine), rightEmpty = emptyLine(rightLine);
    if (leftEmpty && rightEmpty)
        return MoveState::move_forward;
    if (leftEmpty)
        return MoveState::move_left;
    if (rightEmpty)
        return MoveState::move_right;

    double leftRatio = ratio(leftLine), rightRatio = ratio(rightLine);
    if (leftRatio != INFINITY && rightRatio != INFINITY) {
        Vec4i horizon(-CAM_WIDTH / 2, 0, CAM_WIDTH / 2, 0);
        double crossoverX, crossoverY;
        calculateCrossoverPoint(bisector, horizon, crossoverX, crossoverY);
        assert(crossoverY < 1 && crossoverY > -1);
        if (crossoverX > RIGHT_THRESHOLD)
            return MoveState::move_left;
        if (-crossoverX > LEFT_THRESHOLD)
            return MoveState::move_right;
    }
    return move_forward;
}

void controlByNaiveMethod(MoveState state) {
    if (state == move_left)
        turnTo(-TURN_LEFT_ANGLE);
    if (state == move_right)
        turnTo(TURN_RIGHT_ANGLE);
    controlLeft(FORWARD, LEFT_SPEED);
    controlRight(FORWARD, RIGHT_SPEED);
}

void controlByPid(PID &pid, Vec4i &leftLine, Vec4i &rightLine) {
    bool isLeftEmpty = emptyLine(leftLine), isRightEmpty = emptyLine(rightLine);
    if (isLeftEmpty || isRightEmpty) {
        std::cout << "Lines are not detected" << std::endl;
        return;
    }
    // get crossing point of left line and right line
    double crossingX, crossingY;
    calculateCrossoverPoint(leftLine, rightLine, crossingX, crossingY);

    // use distance between crossing x and x of original point as error
    double error = crossingX;
    pid.UpdateError(error);

    // Transfer output to angle
    double totalError = pid.TotalError();
    int angle = (int) totalError;

    std::cout << "total error is " << totalError
              << "\nturn angle is " << angle << std::endl;

    controlLeft(FORWARD, LEFT_SPEED);
    controlRight(FORWARD, RIGHT_SPEED);
    turnTo(angle);
}
