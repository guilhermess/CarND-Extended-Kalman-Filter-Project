#ifndef FUSION_KALMAN_FILTER_HPP
#define FUSION_KALMAN_FILTER_HPP

#include "Eigen/Dense"
#include "Measurement.hpp"
#include "MeasurementRadar.hpp"
#include "MeasurementLaser.hpp"

class FusionKalmanFilter {
 public:
  FusionKalmanFilter();

  /*
   * Process a measurement, returning true if it was successfully processed and the resulting
   * state vector in result.
   * \a result: output state vector
   * \a measurement: input measurement from sensor.
   * \return true if measurement processing is successful, false otherwise
   */
  bool processMeasurement(Eigen::VectorXd &result, const Measurement *measurement);


 private:
  void init(const Measurement *measurement);

  void UpdateF( float delta_t );

  void UpdateQ( float delta_t );

  void Predict();

  void UpdateLaser(const MeasurementLaser *measurement );

  void UpdateRadar(const MeasurementRadar *measurement );

  float getDeltaTime(const Measurement *measurement);

  bool initialized_;

  long long previous_timestamp_;

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement covariance matrix laser
  Eigen::MatrixXd R_laser_;

  // measurement covariance matrix radar
  Eigen::MatrixXd R_radar_;

  //H matrix Laser
  Eigen::MatrixXd H_laser_;

  //H Jacobian Matrix Radar
  Eigen::MatrixXd Hj_;

  float noise_ax_;

  float noise_ay_;

};

#endif
