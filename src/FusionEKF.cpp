#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  /**
  TODO:
    * Finish initializing the FusionEKF.
  */
  R_laser_ << 1, 0,
              0, 1;
  R_radar_ << 1, 0, 0,
              0, 1, 0,
              0, 0, 1;
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  const float PI = 3.1415927;
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1.5, 1.5, 1.5, 1.5;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ro = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      float rodot = measurement_pack.raw_measurements_(2);
      ekf_.x_(0) = ro * cos(phi); // px
      ekf_.x_(1) = ro * sin(phi); // py
      //ekf_.x_(2) = rodot * cos(phi); //vx
      //ekf_.x_(3) = rodot * cos(PI/2 - phi); //vy
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_(0) = measurement_pack.raw_measurements_[0]; 
      ekf_.x_(1) = measurement_pack.raw_measurements_[1];
    }

    //create and initialize state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1;
    ekf_.P_ = ekf_.P_ * 0.1;
    //create state transition matrix
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.Q_ = MatrixXd(4, 4);
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
   */
  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;     //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt3 * dt;
  //1. Modify the F matrix so that the time is integrated
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;
  //2. Set the process covariance matrix Q
  float noise_ax = 5;
  float noise_ay = 5;
  ekf_.Q_ <<  dt4/4*noise_ax, 0, dt3/2*noise_ax, 0,
             0, dt4/4*noise_ay, 0, dt3/2*noise_ay,
             dt3/2*noise_ax, 0, dt2*noise_ax, 0,
             0, dt3/2*noise_ay, 0, dt2*noise_ay;
  //3. Call the Kalman Filter predict() function
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_ * PI*PI /100000.0;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_ * PI*PI /100000.0;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
