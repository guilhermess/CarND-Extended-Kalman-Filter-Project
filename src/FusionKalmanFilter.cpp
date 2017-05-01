/*
 * author: Guilherme Schlinker
 */

#include "FusionKalmanFilter.hpp"
#include "Math.hpp"
#include <iostream>

using namespace Eigen;

FusionKalmanFilter::FusionKalmanFilter() :
    initialized_{false}, previous_timestamp_{0},
    x_{4}, P_{4, 4}, F_{4, 4}, Q_{4, 4},
    R_laser_{2, 2}, R_radar_{3, 3}, H_laser_{2, 4},
    Hj_{3, 4}, noise_ax_{9.0f}, noise_ay_{9.0f} {

  //init state covariance with diagonal matrix
  P_ << 10, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 10, 0,
        0, 0, 0, 1;

  //initialize state transition matrix
  F_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  //initialize laser measurement matrix
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
}

bool FusionKalmanFilter::processMeasurement(VectorXd &result, const Measurement *measurement) {
  if (!measurement->valid())
    return false;

  if (!initialized_) {
    init(measurement);
    return false;
  }

  float dt = getDeltaTime(measurement);

  UpdateF(dt);
  UpdateQ(dt);

  Predict();

  if ( const MeasurementRadar * radar = dynamic_cast<const MeasurementRadar*>(measurement) ) {
    UpdateRadar(radar);
  }
  else if ( const MeasurementLaser * laser = dynamic_cast<const MeasurementLaser*>(measurement) ) {
    UpdateLaser(laser);
  }
  else {
    std::cerr << "Internal Error: Could not recognize measurement as laser or radar" << std::endl;
    return false;
  }

  result = x_;
  return true;
}

float FusionKalmanFilter::getDeltaTime(const Measurement *measurement)
{
  float dt = (measurement->getTimestamp() - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement->getTimestamp();
  return dt;
}

void FusionKalmanFilter::init(const Measurement *measurement) {
  x_ = measurement->getCartesianMeasurement();
  previous_timestamp_ = measurement->getTimestamp();
  initialized_ = true;
}

void FusionKalmanFilter::UpdateF( float delta_t ) {
  F_(0,2) = delta_t;
  F_(1,3) = delta_t;
}

void FusionKalmanFilter::UpdateQ( float delta_t ) {

  Q_ = MatrixXd(4, 4);

  float dt4 = std::pow(delta_t, 4);
  float dt3 = std::pow(delta_t, 3);
  float dt2 = std::pow(delta_t, 2);

  float dax = noise_ax_;
  float day = noise_ay_;

  Q_ << dt4 / 4 * dax, 0, dt3 / 2 * dax, 0,
      0, dt4 / 4 * day, 0, dt3 / 2 * day,
      dt3 / 2 * dax, 0, dt2 * dax, 0,
      0, dt3 / 2 * day, 0, dt2 * day;
}

void FusionKalmanFilter::Predict()  {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void FusionKalmanFilter::UpdateLaser(const MeasurementLaser *measurement) {
  VectorXd z_pred = H_laser_ * x_;
  VectorXd y = measurement->getMeasurement() - z_pred;
  MatrixXd Ht = H_laser_.transpose();
  MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_laser_) * P_;
}

void FusionKalmanFilter::UpdateRadar(const MeasurementRadar *measurement) {
  Hj_ = math::CalculateJacobian(x_);

  float h0 = std::sqrt(std::pow(x_(0), 2) + std::pow(x_(1), 2));
  float h1 = std::atan2(x_(1),x_(0));
  float h2 = (x_(0) * x_(2) + x_(1) * x_(3))/ h0;

  VectorXd z_pred(3);
  z_pred << h0, h1, h2;

  VectorXd y = measurement->getMeasurement() - z_pred;
  y(1) = math::normalizeAngle(y(1));

  MatrixXd Ht = Hj_.transpose();
  MatrixXd S = Hj_ * P_ * Ht + R_radar_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj_) * P_;
}


