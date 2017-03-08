#include <iostream>
#include "tools.h"

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  float elen = estimations.size();
  float glen = ground_truth.size();
  if (elen*glen == 0 || elen != glen) {
    cout << "Sizes are not equal or zeros";
    return rmse;
  }
  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    VectorXd residu = estimations[i].array() - ground_truth[i].array();
    residu = residu.array() * residu.array();
    rmse += residu;
  }
  //calculate the mean and the squared root
  rmse = rmse.array()/elen;
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float divider = sqrt(px*px + py*py);
  float divider2 = divider * divider;
  float divider3 = divider2 * divider;
  //check division by zero
  if (divider == 0) {
      Hj << 0,0,0,0,
            0,0,0,0,
            0,0,0,0;
  }
  //compute the Jacobian matrix
  else {
      Hj << px/divider, py/divider, 0, 0,
            -py/divider2, px/divider2, 0, 0,
            py*(vx*py-vy*px)/divider3, px*(vx*px-vx*py)/divider3, px/divider, py/divider;
  }
  return Hj;
}
