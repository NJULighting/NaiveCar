//
// Created by 訾源 on 2018/11/1.
//
#include "GPIOlib.h"
#include <iostream>

namespace GPIO
{
    int init() {
        std::cout << "Init Car" << std::endl;
        return 0;
    }

    int controlLeft(int direction,int speed) {
        return 0;
    }
    int controlRight(int direction,int speed) {
        return 0;
    }
    int stopLeft() {
        return 0;
    }
    int stopRight() {
        return 0;
    }

    int resetCounter() {
        return 0;
    }
    void getCounter(int *countLeft,int *countRight) {

    }
    int turnTo(int angle) {
        return 0;
    }

    void delay(int milliseconds) {
    }
}

