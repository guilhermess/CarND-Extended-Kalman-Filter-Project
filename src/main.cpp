#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "FusionKalmanFilter.hpp"
#include "GroundTruth.hpp"
#include "Math.hpp"
#include "Measurement.hpp"
#include "MeasurementLaser.hpp"
#include "MeasurementRadar.hpp"

using namespace std;
using Eigen::VectorXd;
using std::vector;

void check_arguments(int argc, char* argv[]) {
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc == 1) {
    cerr << usage_instructions << endl;
  } else if (argc == 2) {
    cerr << "Please include an output file.\n" << usage_instructions << endl;
  } else if (argc == 3) {
    has_valid_args = true;
  } else if (argc > 3) {
    cerr << "Too many arguments.\n" << usage_instructions << endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}

void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char* argv[]) {

  check_arguments(argc, argv);

  string in_file_name_ = argv[1];
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  string out_file_name_ = argv[2];
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  vector<Measurement*> measurements;
  vector<GroundTruth> ground_truths;

  string line;
  while (getline(in_file_, line)) {
    istringstream iss(line);
    long long timestamp;

    string sensor_type;
    iss >> sensor_type;
    if (sensor_type.compare("L") == 0) {
      // LASER MEASUREMENT
      VectorXd measurement{2};
      float x;
      float y;
      iss >> x;
      iss >> y;
      measurement << x, y;
      iss >> timestamp;
      measurements.push_back( new MeasurementLaser(measurement, timestamp));
    } else if (sensor_type.compare("R") == 0) {
      // RADAR MEASUREMENT
      VectorXd measurement{3};
      float ro;
      float phi;
      float ro_dot;
      iss >> ro;
      iss >> phi;
      iss >> ro_dot;
      measurement << ro, phi, ro_dot;
      iss >> timestamp;
      measurements.push_back(new MeasurementRadar(measurement, timestamp));
    }

    // read ground truth data to compare later
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;
    VectorXd value{4};
    value << x_gt, y_gt, vx_gt, vy_gt;
    ground_truths.push_back(GroundTruth(value));
  }

  // Create a Fusion EKF instance
  FusionKalmanFilter fusion_kalman_filter;

  // used to compute the RMSE later
  vector<VectorXd> estimation_vector;
  vector<VectorXd> ground_truth_vector;

  //Call the EKF-based fusion
  size_t measurements_size = measurements.size();
  for (size_t i = 0; i < measurements_size; ++i) {
    Measurement *measurement = measurements[i];
    VectorXd result(4);
    if ( !fusion_kalman_filter.processMeasurement(result, measurement) ) {
      continue;
    }

    // output the estimation
    out_file_ << result(0) << "\t";
    out_file_ << result(1) << "\t";
    out_file_ << result(2) << "\t";
    out_file_ << result(3) << "\t";

    // output the measurements
    VectorXd cartesian_measurement = measurement->getCartesianMeasurement();
    out_file_ << cartesian_measurement(0) << "\t";
    out_file_ << cartesian_measurement(1) << "\t";

    // output the ground truth packages
    GroundTruth ground_truth = ground_truths[i];
    out_file_ << ground_truth.getValue()(0) << "\t";
    out_file_ << ground_truth.getValue()(1) << "\t";
    out_file_ << ground_truth.getValue()(2) << "\t";
    out_file_ << ground_truth.getValue()(3) << "\n";

    estimation_vector.push_back(result);
    ground_truth_vector.push_back(ground_truth.getValue());
  }

  // compute the accuracy (RMSE)
  cout << "Accuracy - RMSE:" << endl << math::CalculateRMSE(estimation_vector, ground_truth_vector) << endl;

  // close files
  if (out_file_.is_open()) {
    out_file_.close();
  }

  if (in_file_.is_open()) {
    in_file_.close();
  }

  for ( auto measurement : measurements )
    delete measurement;

  return 0;
}
