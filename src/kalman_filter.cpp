#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &Hj_in, MatrixXd &R_in,
                        MatrixXd &R_in_ekf, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  Hj_ = Hj_in;
  R_ = R_in;
  R_ekf_ = R_in_ekf;
  Q_ = Q_in;

  I_ = MatrixXd::Identity(4, 4);
}

/**
 * Predicts the state and updates the state-covariance matrix
 * according to current state-transition matrix.
 */
void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  // New estimate
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  // Avoid dividing by zero and skip update-step when position coordinate px and py are zeros.
  if (0. == px || 0. == py)
    return;

  // Calculate the Jacobian (linearized measurement).
  Hj_ = tools.CalculateJacobian(x_);
  VectorXd h(3);
  float rho = sqrt(px * px + py * py);
  h << rho, atan2(py, px), (px * vx + py * vy)/rho;

  VectorXd y = z - h;
  if(y[1] > M_PI)
    y[1] -= 2.f * M_PI;
  if(y[1] < -M_PI)
    y[1] += 2.f * M_PI;

  MatrixXd Hjt = Hj_.transpose();
  MatrixXd S = Hj_*P_*Hjt + R_ekf_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_*Hjt*Si;

  // New estimate
  x_ = x_ + (K * y);
  P_ = (I_ - K * Hj_) * P_;
}

