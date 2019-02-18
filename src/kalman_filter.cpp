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

/* The Predict function remains same for both Lidar and Radar
 * sensor measurement inputs
 * F_j is equal to F, as prediction is based on linear model. */
void KalmanFilter::Predict()
	{
		x_ = F_ * x_ ;
		P_ = F_ * P_ * F_.transpose() + Q_ ;
	}

/*This Update function will be called for measurement update
 * operation for a Lidar sensor, as the inputs are Cartesian
 * co-ordinates, and no conversions or jacobians are required. */

void KalmanFilter::Update(const VectorXd &z)
	{

		VectorXd y = z - (H_ * x_);
		MatrixXd S = (H_ * P_ * H_.transpose()) + R_;
		MatrixXd K = P_ * H_.transpose() * S.inverse();

		//new estimate
		x_ = x_ + (K * y);
		int x_size = x_.size();
		MatrixXd I = MatrixXd::Identity(x_size, x_size);
		P_ = (I - K * H_) * P_;
	}

/* This UpdateEKF function will be called for measurements from
 * radar is received, as the inputs are in polar form. Here, the
 * conversions and computation of Jacobians are required. */

void KalmanFilter::UpdateEKF(const VectorXd &z)
	{
		VectorXd hx = VectorXd(3);
		auto px = x_[0];
		auto py = x_[1];
		auto vx = x_[2];
		auto vy = x_[3];

		// Make sure that both the px and py are not equal to zero
		if(px == 0 && py == 0)
			{
				px = 0.00001;
				py = 0.00001;
			}

		hx[0] = sqrt(px*px+py*py);							// rho
		hx[1] = atan2(py,px);								// phi
		hx[2] = (px * vx + py * vy) / (sqrt(px*px+py*py));	// rho_dot

		VectorXd y = z - hx;

		// The value of phi should be between -pi and pi
		y[1] = atan2(sin(y[1]),cos(y[1]));

		MatrixXd S = H_ * P_* H_.transpose() + R_;
		MatrixXd K = P_ * H_.transpose() * S.inverse();

		x_ = x_ + (K * y);
		int x_size = x_.size();
		MatrixXd I = MatrixXd::Identity(x_size,x_size);
		P_ = (I - K * H_) * P_;

	}
