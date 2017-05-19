#include "PID.h"
#include <math.h>
#include <iostream>
#include <random>
constexpr double pi() { return M_PI; }

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double drift) {
    p[0] = Kp;
    p[1] = Ki;
    p[2] = Kd;
    dp[0] = 1;
    dp[1] = 1;
    dp[2] = 1;
    steering_drift = drift;
    p_error = 0;
    i_error = 0;
    d_error = 0;
    d_rate = 0;
    total_error = 0;
    best_err = 1e9;
    numSteps = 0;
    indx = 0;
    first_run = true;
    second_run = false;
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    d_rate = p_error != 0 ? d_error/p_error : d_error;
    p_error = cte;
    i_error += cte;
    if (numSteps > N) {
        total_error += pow(cte, 2);
    }
    numSteps++;
}

double PID::TotalError() {
    return total_error/(numSteps - N);
}

double PID::GetSteerValue() {

    double value = -p[0] * p_error - p[2] * d_error - p[1] * i_error;
    double max_angle = pi() /3.50;
    value = value < -max_angle ? -max_angle : value;
    value = value > max_angle ? max_angle : value;
    std::default_random_engine gen;
    std::normal_distribution<double> gauss(value, 0);
    value = gauss(gen);
    
    return value;
}

double PID::GetSpeedValue() {

    double value = -p[0] * p_error - p[2] * d_error - p[1] * i_error;
    value = value < -1 ? -1 : value;
    value = value > 1 ? 1 : value;
    std::default_random_engine gen;
    std::normal_distribution<double> gauss(value, 0);
    value = gauss(gen);
    return value;
}

double PID::GetCTERate() {
    return abs(d_rate);
}

void PID::twiddle() {
    double err = TotalError();
    double sumdp = dp[0] + dp[1] + dp[2];
    if (sumdp > TOL) {
        if (first_run && !second_run) {
            p[indx] += dp[indx];
            first_run = false;
            second_run = false;
            cout << "first +=: p[" << indx << "] = " << p[indx] << endl;
        }
        if ((err < best_err) && !first_run && !second_run) {
            best_err = err;
            dp[indx] *= 1.1;
            first_run = true;
            second_run = false;
            cout << "last *=1.1: p[" << indx << "] = " << p[indx] << endl;
            indx = (++indx)%3;
        } else {
            p[indx] -= 2 * dp[indx];
            first_run = false;
            second_run = true;
            cout << "second -=2: p[" << indx << "] = " << p[indx] << endl;
        }
        if ((err < best_err) && !first_run && second_run) {
            best_err = err;
            dp[indx] *= 1.1;
            first_run = true;
            second_run = false;
            cout << "last *=1.1: p[" << indx << "] = " << p[indx] << endl;
            indx = (++indx)%3;
        } else {
            p[indx] += dp[indx];
            dp[indx] *= 0.9;
            first_run = true;
            second_run = false;
            cout << "last *=0.9: p[" << indx << "] = " << p[indx] << endl;
            indx = (++indx)%3;
        }
    }
}


