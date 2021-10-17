// CREDIT: Extended Kalman Filters lesson: Laser Measurements Part 4, kalman_filter.cpp code, https://github.com/asuri2/CarND-Extended-Kalman-Filter-P6/blob/master/src/kalman_filter.cpp (used as reference)

#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}
// CREDIT: Extended Kalman Filters lesson: Laser Measurements Part 4, kalman_filter.cpp code
void KalmanFilter::Predict()
{
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}
// CREDIT: Extended Kalman Filters lesson: Laser Measurements Part 4, kalman_filter.cpp code
void KalmanFilter::Update(const VectorXd &z)
{
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

// Used the same code as the above funtion which as provided here Extended Kalman Filters lesson: Laser Measurements Part 4, kalman_filter.cpp code, and the Extended Kalman filter lesson 24
// I simply added the EKF functions, according to the EKF algortithims generalization, the KF and the EKF are: 
// "Although the mathematical proof is somewhat complex, it turns out that the Kalman filter equations and extended Kalman filter equations are very similar."
// This will recaculate rho, phi, and rho_dot
void KalmanFilter::UpdateEKF(const VectorXd &z)
{
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  // These are the values present in the simulator
  double x_value, y_value, vx, vy;
  
  // These are the variables explained in the videos:
      // rho - RANGE: radial distance from origin 
      // phi - BEARING: angle between phi and x
      // rho_dot - RADIAL VELOCITY: change of phi (range rate)
  double rho, phi, rho_dot;
  
  // Setting simulator values here
  x_value = x_(0);
  y_value = x_(1);
  vx = x_(2);
  vy = x_(3);
  
  // Recalculating here
  rho = sqrt(x_value * x_value + y_value * y_value);
  phi =  atan2(y_value, x_value);
  rho_dot = (x_value * vx + y_value * vy) / rho;
  
  // Keeps track of the estimated values
  VectorXd recalculated = VectorXd(3);
  recalculated << rho, phi, rho_dot;

  VectorXd z_pred = H_ * x_;
  //"The HH matrix from the lidar lesson and h(x)h(x) equations from the radar lesson are actually accomplishing the same thing;
  // they are both needed to solve y = z - Hx'y=z−Hx n the update step." - Radar measurement lesson
  VectorXd y = z - recalculated;
  
  // If we want to normalize the angles between pi we can do the following
  while(y(1) > 3.14)
  {
    y(1) -= 2 * 3.14;
  }
  while(y(1) < -3.14)
  {
    y(1) += 2 * 3.14;
  }
  
  // The following code was given in the lesson 
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}