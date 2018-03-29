#include <iostream>
#include "kalman_filter.h"
#include "tools.h"
using namespace std;

KalmanFilter::KalmanFilter() {
  x_ = VectorXd(4);
  x_.fill(0.0);
  P_  = MatrixXd(4, 4);
  P_.fill(0.0);
  F_ = MatrixXd(4, 4);
  F_.fill(0.0);
  Q_ = MatrixXd(4, 4);
  Q_.fill(0.0);
  NIS_radar_ = 0;
  NIS_laser_ = 0;
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht; 
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  //calculate NIS
  NIS_laser_ = y.transpose() * S.inverse() * y;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  Tools tools;
  MatrixXd Hj_ = MatrixXd(3, 4);
  Hj_ = tools.CalculateJacobian(x_);
  VectorXd hx = VectorXd(3);
  int iszero = x_(0) + x_(1);
  if (iszero!=0) {
    hx << sqrt(x_(0)*x_(0) + x_(1)*x_(1)),
        atan2(x_(1), x_(0)),
        (x_(0)*x_(2) + x_(1)*x_(3))/sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    VectorXd y = z - hx;
    MatrixXd Ht = Hj_.transpose();
    MatrixXd PHt = P_ * Ht; 
    MatrixXd S = Hj_ * PHt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = PHt * Si;
    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * Hj_) * P_;
    //calculate NIS
    NIS_radar_ = y.transpose() * S.inverse() * y;
  } 
}
