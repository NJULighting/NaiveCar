//
// Created by 訾源 on 2018/11/1.
//
#include <iostream>
#include <opencv2/opencv.hpp>

#include "GPIOlib.h"

#define _DEBUG

using namespace GPIO;
using namespace cv;

const std::string CAM_PATH = "/dev/video0";
const int CAM_HEIGHT = 480;
const int CAM_WIDTH = 640;

const int MAX_SPEED = 100;
const int TURN_SPEED = 50;

const double THRESHOLD = 0.03;

int main()
{
    VideoCapture capture(CAM_PATH);
    if (!capture.isOpened()) {
        capture.open(atoi(CAM_PATH.c_str()));
    }
    init();
    turnTo(0);

    int state = -1;
    int cycle = 0;

    Mat image, imageLeft, imageRight;
    Rect roiL(0, 0, CAM_WIDTH / 2, CAM_HEIGHT);
    Rect roiR(CAM_WIDTH / 2, 0, CAM_WIDTH / 2, CAM_HEIGHT);
    while (true) {
        capture >> image;
        if (image.empty())
            break;
        if (cycle < 90) {
            if (cycle == 0)
                std::cout << "Fasten seat belt..." << std::endl;
            if (cycle % 30 == 0)
                std::cout << (3 - cycle / 30) << "..." << std::endl; // 倒计时可还行
            cycle++;
            continue;
        }
        cvtColor(image, image, CV_BGR2GRAY);
        threshold(image, image, 80, 255, THRESH_BINARY); // 转化成灰度图像之后二值化

        imageLeft = image(roiL);
        imageRight = image(roiR);
        double rateL = 1 - countNonZero(imageLeft) * 2.0 / CAM_HEIGHT / CAM_WIDTH;
        double rateR = 1 - countNonZero(imageRight) * 2.0 / CAM_WIDTH / CAM_HEIGHT;

        #ifdef _DEBUG
        std::cout << "L = " << rateL << ", R = " << rateR << std::endl;
        #endif

        if (rateL < THRESHOLD && rateR < THRESHOLD && state != 0) {
            controlLeft(FORWARD, MAX_SPEED);
            controlRight(FORWARD, MAX_SPEED);
            state = 0;
        } else if (rateL >= THRESHOLD && rateR < THRESHOLD && state != 1) {
            init();
            turnTo(5);
            controlRight(BACKWARD, TURN_SPEED);
            state = 1;
        } else if (rateR >= THRESHOLD && rateL < THRESHOLD && state != 2) {
            init();
            turnTo(-5);
            controlLeft(BACKWARD, TURN_SPEED);
            state = 2;
        } else if (rateR >= THRESHOLD && rateL >= THRESHOLD) {
            init();
            turnTo(0);
            controlLeft(FORWARD, MAX_SPEED);
            controlRight(FORWARD, MAX_SPEED);
            delay(500);
            init();
            return 0;
        }
        waitKey(1);
    }
    return 0;
}
