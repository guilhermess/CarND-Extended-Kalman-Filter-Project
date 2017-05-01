#ifndef MATH_HPP_
#define MATH_HPP_

#include <vector>
#include "Eigen/Dense"

namespace math {

/**
* A helper method to calculate RMSE.
*/
Eigen::VectorXd
CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

/**
* A helper method to calculate Jacobians.
*/
Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd &x_state);

/*
 * Normalize angle in radians between -PI and PI
 */
float normalizeAngle(float angleRadian);


}

#endif