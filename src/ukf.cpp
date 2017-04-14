#include <iostream>
#include "ukf.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  debug = true;
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;
  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;
  // initial state vector
  x_ = VectorXd(5);
  x_pre = VectorXd(5);
  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_pre = MatrixXd(5, 5);
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;//5~2
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;//0.5 ~ 0.2
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;//0.015;
  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15; //0.015;
  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;
  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03; //0.0175;
 // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3; //0.1;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_ = false;
  time_us_ = 0;
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_/(lambda_ + n_aug_);
  for (int i = 1; i < 2*n_aug_+1; i++) {
    weights_(i) = 1/(2*(lambda_ + n_aug_));
  }
  x_ = VectorXd(n_x_);
  P_ = MatrixXd(n_x_, n_x_);
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  // NIS_ = (z - z_pred_).transpose() * S.inverse() * (z - z_pred_)
  //expected_NIS_radar_ = 7.815;
  //expected_NIS_laser_ = 5.991;
  NIS_radar_ = 0;
  NIS_laser_ = 0;
  rcount = 0;
  lcount = 0;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    x_.fill(0.001);
    if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      float ro = meas_package.raw_measurements_(0);
      float phi = meas_package.raw_measurements_(1);
      float rodot = meas_package.raw_measurements_(2);
      x_(0) = ro * cos(phi); // px
      x_(1) = ro * sin(phi); // py
      // done initializing, no need to predict or update
      is_initialized_ = true;
      //debug
      rcount += 1;
    }
    else if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_(0) = meas_package.raw_measurements_[0]; 
      x_(1) = meas_package.raw_measurements_[1];
      is_initialized_ = true;
      //debug
      lcount += 1;
    } 
    //create and initialize state covariance matrix P
    P_ <<  1, 0, 0, 0, 0,
           0, 1, 0, 0, 0,
           0, 0, 1, 0, 0,
           0, 0, 0, 1, 0,
           0, 0, 0, 0, 1;
    P_ = P_ * 1;
    time_us_ = meas_package.timestamp_;
    return;
  } 

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  
  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0; //dt - expressed in seconds
  time_us_ = meas_package.timestamp_;
  
  try {
    while (dt > 0.1)
    {
      const double delta_t = 0.05;
      Prediction(delta_t);
      dt -= delta_t;
    }
    Prediction(dt);
  } catch (std::range_error e) {
      // If convariance matrix is non positive definite (because of numerical instability),
      // restart the filter using previous measurement as initialiser.
      x_ = x_pre;
      P_ = P_pre;
      // Redo prediction using the current measurement
      // We don't get exception this time, because initial P (identity) is positive definite.
      Prediction(dt);
  }
  x_pre = x_;
  P_pre = P_;

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  VectorXd z = meas_package.raw_measurements_;
  if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      UpdateRadar(meas_package);
      rcount += 1;
  }
  if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) {
      UpdateLidar(meas_package);
      lcount += 1;
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  /* 
  1. AugmentedSigmaPoints 
  Input: x_, P_, std_a_, std_yawdd_
  Output: Xsig_aug
  */
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  //create aug sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1); 

  //cout << x_ << endl;
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
  //create augmented covariance matrix
  P_aug.fill(0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_aug_-2, n_aug_-2) = std_a_*std_a_;
  P_aug(n_aug_-1, n_aug_-1) = std_yawdd_*std_yawdd_;
  //create square root matrix
  Eigen::LLT<MatrixXd> lltOfPaug(P_aug);
  if (lltOfPaug.info() == Eigen::NumericalIssue) {
      // if decomposition fails, we have numerical issues
      std::cout << "LLT failed!" << std::endl;
      //Eigen::EigenSolver<MatrixXd> es(P_aug);
      //cout << "Eigenvalues of P_aug:" << endl << es.eigenvalues() << endl;
      throw std::range_error("LLT failed");
  }
  // 2. get the lower triangle
  MatrixXd L = lltOfPaug.matrixL();
  //MatrixXd L = P_aug.llt().matrixL();
  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for(int i=0; i < n_aug_; i++){
      Xsig_aug.col(i+1)=x_aug + sqrt(lambda_+n_aug_) * L.col(i);
      Xsig_aug.col(i+1+n_aug_)=x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }
  /* 
  2. SigmaPointPrediction 
  Input: Xsig_aug, delta_t
  Output: Xsig_pred_
  */
  for (int i = 0; i< 2*n_aug_+1; i++) {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.0001) {
        px_p = p_x + v/yawd * ( sin (yaw+yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }
    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
  /* 
  3. PredictMeanAndCovariance
  Input: Xsig_pred_, weights_
  Output: (predicted) x_, P_
  */
  //predict state mean
  x_ = Xsig_pred_ * weights_;
  //predict state covariance matrix
  P_.fill(0);
  for (int i=0; i<2*n_aug_+1; i++){
    MatrixXd x_diff = Xsig_pred_.col(i) - x_;
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}
  
/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  /* 
  1. PredictLidarMeasurement
  Input: Xsig_pred_, weights_, std_radr_, std_radphi, std_radrd
  Output: z_pred, S, Zsig
  */
  int n_z = 2;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  //create matrix for Kalman gain K
  MatrixXd K = MatrixXd(n_x_, n_z);
  //transform sigma points into measurement space
  for (int i=0; i<2*n_aug_+1; i++) {
    Zsig(0,i) = Xsig_pred_(0,i); //px
    Zsig(1,i) = Xsig_pred_(1,i); //py
  }
  //calculate mean predicted measurement
  z_pred.fill(0.0);
  z_pred = Zsig * weights_;
  //for (int i=0; i < 2*n_aug_+1; i++) {
  //    z_pred = z_pred + weights_(i) * Zsig.col(i);
  //}
  //std::cout << z_pred;
  //calculate measurement covariance matrix S
  S.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++){
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_laspx_ * std_laspx_, 0,
       0, std_laspy_ * std_laspy_;
  S += R;
  /* 
  2. UpdateState
  Input: Xsig_pred_, z_pred, x_, P_, weights_, (measured) z
  Process: Tc, K, S
  Output: (updated) x_, P_
  */
  VectorXd z = meas_package.raw_measurements_;
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++){
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  //calculate Kalman gain K;
  K = Tc * S.inverse();
  //residual
  VectorXd z_diff = z - z_pred;
  
  //update state mean and covariance matrix
  x_ = x_ + K*z_diff;
  P_ = P_ - K*S*K.transpose();
  //calculate NIS
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  /* 
  1. PredictRadarMeasurement
  Input: Xsig_pred_, weights_, std_radr_, std_radphi, std_radrd
  Output: z_pred, S, Zsig
  */
  int n_z = 3;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  //create matrix for Kalman gain K
  MatrixXd K = MatrixXd(n_x_, n_z);
  //transform sigma points into measurement space
  for (int i=0; i<2*n_aug_+1; i++) {
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double vx = cos(yaw)*v;
    double vy = sin(yaw)*v;

    if (p_x == 0 && p_y == 0) {
      Zsig(0,i) = 0;
      Zsig(1,i) = 0;
      Zsig(2,i) = 0;
    } else {
      Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);
      Zsig(1,i) = atan2(p_y, p_x);
      Zsig(2,i) = (p_x*vx + p_y*vy)/sqrt(p_x*p_x + p_y*p_y);
    }
  }
  //calculate mean predicted measurement
  z_pred.fill(0.0);
  z_pred = Zsig * weights_;
  //for (int i=0; i < 2*n_aug_+1; i++) {
  //    z_pred = z_pred + weights_(i) * Zsig.col(i);
  //}
  //std::cout << z_pred;
  //calculate measurement covariance matrix S
  S.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++){
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_ * std_radr_, 0, 0,
       0, std_radphi_ * std_radphi_, 0,
       0, 0, std_radrd_ * std_radrd_;
  S += R;

  /* 
  2. UpdateState
  Input: Xsig_pred_, z_pred, x_, P_, weights_, (measured) z
  Process: Tc, K, S
  Output: (updated) x, P
  */
  VectorXd z = meas_package.raw_measurements_;
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++){
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  //calculate Kalman gain K;
  K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //update state mean and covariance matrix
  x_ = x_ + K*z_diff;
  P_ = P_ - K*S*K.transpose();
  //calculate NIS
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}
