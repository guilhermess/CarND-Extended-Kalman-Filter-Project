
#include "Math.hpp"
#include <iostream>

namespace math {

using namespace Eigen;

VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                              const std::vector<Eigen::VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if ((estimations.size() == 0) || (estimations.size() != ground_truth.size()))
  {
    std::cerr << "Could not calculate RMSE: estimations empty or not the same size as ground truth vector"
              << std::endl;
    return rmse;
  }

  //accumulate squared residuals
  for (int i = 0; i < estimations.size(); ++i)
  {
    VectorXd estimated = estimations[i];
    VectorXd truth = ground_truth[i];
    VectorXd residual = estimated - truth;
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  //calculate the mean
  rmse = rmse / estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

MatrixXd CalculateJacobian(const Eigen::VectorXd &x_state) {
  MatrixXd Hj(3, 4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //check division by zero
  if (px == 0 || py == 0)
  {
    std::cerr << "Error: px or py is 0, cannot calculate Jacobian" << std::endl;
    return Hj;
  }

  float px2 = std::pow(px, 2.0f);
  float py2 = std::pow(py, 2.0f);
  float px2_py2 = px2 + py2;
  float sqrt_px2_py2 = std::sqrt(px2_py2);
  float pow_px2_py2_1p5 = std::pow(px2_py2, 1.5f);

  float a11 = px / sqrt_px2_py2;
  float a12 = py / sqrt_px2_py2;
  float a21 = -py / px2_py2;
  float a22 = px / px2_py2;
  float a31 = py * (vx * py - vy * px) / pow_px2_py2_1p5;
  float a32 = px * (vy * px - vx * py) / pow_px2_py2_1p5;
  float a33 = px / sqrt_px2_py2;
  float a34 = py / sqrt_px2_py2;

  //compute the Jacobian matrix
  Hj << a11, a12, 0, 0,
      a21, a22, 0, 0,
      a31, a32, a33, a34;

  return Hj;
}

float normalizeAngle(float angleRadian) {
  while ( angleRadian < -M_PI )
    angleRadian += 2*M_PI;
  while ( angleRadian > M_PI )
    angleRadian -= 2*M_PI;
  return angleRadian;
}

}


