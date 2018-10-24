//
// Created by Junda Liao on 2018/10/24.
// This program was copied from https://github.com/parilo/CarND-PID-Control-Project
//

#ifndef NAIVECAR_PID_H
#define NAIVECAR_PID_H


class PID {
public:
    /*
    * Errors
    */
    double p_error;
    double i_error;
    double d_error;

    /*
    * Coefficients
    */
    double Kp;
    double Ki;
    double Kd;

    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();

private:

    double prev_cte;
    double prev_2_cte;

};


#endif //NAIVECAR_PID_H
