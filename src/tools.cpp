#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (0 == estimations.size() || estimations.size() != ground_truth.size()) {
    return rmse;
  }

  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    VectorXd res = estimations[i] - ground_truth[i];
    res = res.array() * res.array();
    rmse += res;
  }

  //calculate the mean
  rmse = rmse / estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float p_square = px*px + py*py;
  float p_sqrt = sqrt(p_square);
  float p_cubrt = p_square * p_sqrt;

  //check division by zero
  if (fabs(p_square) < 0.0001) {
    cout << "CalculateJacobian() - Error - Division by Zero\n";
    return Hj;
  }
  //compute the Jacobian matrix
  Hj << px/p_sqrt, py/p_sqrt, 0, 0,
      -py/p_square, px/p_square, 0, 0,
      py*(vx*py - vy*px)/p_cubrt, px*(vy*px - vx*py)/p_cubrt, px/p_sqrt, py/p_sqrt;

  return Hj;
}
