#!/bin/bash
g++ $1 -o test `pkg-config --cflags --libs opencv` -L. -lwiringPi -lGPIO –lpthread