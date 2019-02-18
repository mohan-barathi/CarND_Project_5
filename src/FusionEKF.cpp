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

  //measurement function for Laser input
  H_laser_ << 1,0,0,0,
              0,1,0,0;



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
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: x_ is being initialized" << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1.0, 1.0, 1.0, 1.0;

    // Dummy F and Q values, that are passed as arguments to initialize ekf
    // These values will be updated on each cycle.
    MatrixXd F = MatrixXd(4,4);
    MatrixXd Q = MatrixXd(4,4);
    MatrixXd P = MatrixXd(4,4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        auto x_polor = measurement_pack.raw_measurements_;
        ekf_.x_[0] = x_polor[0]*cos(x_polor[1]);
        ekf_.x_[1] = x_polor[0]*sin(x_polor[1]);
        ekf_.Init(ekf_.x_,P,F,Hj_,R_radar_,Q);

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
    	ekf_.x_[0] = measurement_pack.raw_measurements_[0];
    	ekf_.x_[1] = measurement_pack.raw_measurements_[1];
		ekf_.Init(ekf_.x_,P,F,H_laser_,R_laser_,Q);
    }

    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    cout << "initialization is complete..!" << endl;
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

  // As the elapsed timestamp is in ticks, convert it to seconds
  auto del_t  = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;

  /* Predict Function will be called only if the measurements received at
  ** different time */
  if (del_t > 0)
  {
	  // state transition matrix
	  ekf_.F_ << 1, 0, del_t, 0,
			  	 0, 1, 0, del_t,
				 0, 0, 1, 0,
				 0, 0, 0, 1 ;

	  // noise_ax = 9 and noise_ay = 9 for Q matrix
	  auto sig_ax = 9;
	  auto sig_ay = 9;
	  auto del_t2 = del_t * del_t;
	  auto del_t3 = del_t * del_t2;
	  auto del_t4 = del_t * del_t3;

	  // process noise co-variance matrix
	  ekf_.Q_ << (del_t4 * sig_ax * 0.25), 0, (del_t3 * sig_ax * 0.5), 0,
			  	 0, (del_t4 * sig_ay * 0.25), 0, (del_t3 * sig_ay * 0.5),
				 (del_t3 * sig_ax * 0.5), 0, (del_t2 * sig_ax), 0,
				 0, (del_t3 * sig_ay * 0.5), 0, (del_t2 * sig_ay);

	  ekf_.Predict();
  }

  /**
   * Measurement Update
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
	  Hj_ = tools.CalculateJacobian(ekf_.x_);
	  ekf_.H_ = Hj_;
	  ekf_.R_ = R_radar_;
	  ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  }

  else
  {
	  ekf_.H_ = H_laser_;
	  ekf_.R_ = R_laser_;
	  ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
