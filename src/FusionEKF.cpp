#include "FusionEKF.h"
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

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  noise_ax = 9.;
  noise_ay = 9.;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    // first measurement
    cout << "EKF: " << endl;
    VectorXd x(4);
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ro = measurement_pack.raw_measurements_[0];
      float theta = measurement_pack.raw_measurements_[1];

      x << ro * cos(theta), ro * sin(theta), 0., 0.;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1],
            0., 0.;
    }

    // Initialize Kalman filter matrices.
    // State uncertainty covariance matrix
    MatrixXd P_in(4, 4);
    P_in << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

    // State transition matrix
    MatrixXd F_in(4, 4);
    F_in << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

    // Process covariance matrix, will get updated while processing a measurement.
    MatrixXd Q_in(4, 4);

    H_laser_ << 1, 0, 0, 0,
        0, 1, 0, 0;

    previous_timestamp_ = measurement_pack.timestamp_;
    ekf_.Init(x, P_in, F_in, H_laser_, Hj_, R_laser_, R_radar_, Q_in);

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // Compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;  //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  if (dt > 0.) {
    ekf_.F_(0,2) = dt;
    ekf_.F_(1,3) = dt;

    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_2 * dt_2;
    float dt_3_by_2 = dt_3 / 2.;
    float dt_4_by_4 = dt_4 / 4.;

    ekf_.Q_ << dt_4_by_4 * noise_ax, 0, dt_3_by_2 * noise_ax, 0,
        0, dt_4_by_4 * noise_ay, 0, dt_3_by_2 * noise_ay,
        dt_3_by_2 * noise_ax, 0, dt_2 * noise_ax, 0,
        0, dt_3_by_2 * noise_ay, 0, dt_2 * noise_ay;

    ekf_.Predict();
  }

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  else {
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
