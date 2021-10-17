// CREDIT: https://github.com/felipemartinezs/CarND-Extended-Kalman-Filter-Project/blob/master/src/FusionEKF.cpp (used as reference), Extended Kalman Filter lesson, project code lesson, tracking.cpp from laster measurements part 4, (Credit: Radar Measuremnts video lesson)

#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF()
{
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
  
  // Measurement of the H_laser 2x4 matrix
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // Measurement of the Hj_ 3x4 matrix
  Hj_ << 1, 1, 0, 0,
         1, 1, 0, 0,
         1, 1, 1, 1; 
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
  /**
   * Initialization
   */
  if (!is_initialized_)
  {
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    
    // TODO: Initialize the state ekf_.x_ with the first measurement.
    ekf_.P_ = MatrixXd(4, 4);
    // TODO: Create the covariance matrix.
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 1,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      cout << "Kalman Filter Initialization " << endl;
      // TODO: Initialize state.
      //(Credit: Radar Measuremnts video lesson)
      float rho = measurement_pack.raw_measurements_(0); // RANGE: radial distance from origin 
	  float phi = measurement_pack.raw_measurements_(1); // BEARING: angle between phi and x
	  float rho_dot = measurement_pack.raw_measurements_(2); // RADIAL VELOCITY: change of phi (range rate)
      
      // TODO: Convert radar from polar to cartesian coordinates and initialize state.
      // To convert to from from polar to cartesian we need to multiply our values by sin and/or cos of teh range phi
      // Since this is a matrix of 4 we need to multiply each entry by this step
      cout << "Converting from polar to cartesian coordinates " << endl;
      ekf_.x_(0) = rho * cos(phi);
      ekf_.x_(1) = rho * sin(phi);      
      ekf_.x_(2) = rho_dot * cos(phi);
      ekf_.x_(3) = rho_dot * sin(phi);

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
      // TODO: Initialize state.
      // set the state with the initial location and zero velocity
      ekf_.x_ << measurement_pack.raw_measurements_[0], 
                 measurement_pack.raw_measurements_[1], 
                 0, 
                 0;
    }
	previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  // CREDIT: For the following section of code I referred to the tracking.cpp file in the extended kalman filter section
  
  // Update the state transition matrix F according to the new elapsed time.
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
  
  // Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
  float noise_ax = 9;
  float noise_ay = 9;
  
  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // TODO: YOUR CODE HERE
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
              0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
              dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
              0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
  
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */
  
  // Make a tools object
  // CREDIT: Used the tracking.cpp code from the Extended Kalman Filter lesson for this section
  // Creating a myTools reference so I can use the calculate jacobian finction
  Tools myTools;
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    // TODO: Radar updates
    // Update the state and covariance matrices.
    // The ekf_ variable is an instance of the KalmanFilter class.
    cout << "Updating radar " << endl;
    ekf_.R_ = R_radar_; 
    Hj_ = myTools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    // Use the sensor type to perform the update step.
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } 
  else
  {
    // TODO: Laser updates
    cout << "Updating laser " << endl;
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
	ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
