//
// Created by Junda Liao on 2018/10/24.
// This program was copied from https://github.com/parilo/CarND-PID-Control-Project
//

#include "PID.h"

#include "PID.h"


/*
* TODO: Complete the PID class.
*/

PID::PID() {
    prev_cte = 0;
    prev_2_cte = 0;

    p_error = 0;
    i_error = 0;
    d_error = 0;
}

PID::~PID() = default;

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void PID::UpdateError(double cte) {
    p_error = cte;

    // more accurate derivative
    // see https://en.wikipedia.org/wiki/Finite_difference_coefficient
    d_error = 1.5 * cte - 2 * prev_cte + 0.5 * prev_2_cte;
    prev_2_cte = prev_cte;
    prev_cte = cte;

    i_error += cte;
}

double PID::TotalError() {
    return -(Kp * p_error + Kd * d_error + Ki * i_error);
}