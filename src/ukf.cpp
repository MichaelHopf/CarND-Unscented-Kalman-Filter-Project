#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  iteration = 0;
  n_x_ = 5;
  n_aug_ = 7;
  EPS = pow(10, -5);

  lambda_ = n_aug_ - 3;

  MatrixXd Xsig_pred(n_x_, 2 * n_aug_ + 1);
  weights_ = VectorXd(2 * n_aug_ + 1);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
	  if (i == 0) {
		  weights_(i) = lambda_ / (lambda_ + n_aug_);
	  }
	  else {
		  weights_(i) = 1 / (2 * (lambda_ + n_aug_));
	  }
  }

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:
  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
	double delta_t;
	
	if (is_initialized_ == false) {
		cout << "Here we initialize!\n";
		// Initialize x_
		if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
		{
			const double rho = meas_package.raw_measurements_(0);
			const double phi = meas_package.raw_measurements_(1);
			x_ << rho * cos(phi), rho * sin(phi), 0, 0, 0;
		}
		else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
		{
			x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0, 0, 0;
		}

		//Initialize P_
		P_ << 1, 0, 0, 0, 0,
			0, 1, 0, 0, 0,
			0, 0, 5, 0, 0,
			0, 0, 0, 1, 0,
			0, 0, 0, 0, 1;
		time_us_ = meas_package.timestamp_;
		cout << "Initialized time is " << time_us_ << "\n";
		is_initialized_ = true;
		//Prediction(0.1);	
	}
	else {
		long long time_stamp = meas_package.timestamp_;
		delta_t = (time_stamp - time_us_) / 1000000.0;
		time_us_ = time_stamp;
		iteration += 1;
		cout << "\n" << "We are in iteration " << iteration << "\n";
		Prediction(delta_t);
	
	
	// Update step
	if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && (use_laser_ == true)) {
		UpdateLidar(meas_package);
	}
	else if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && (use_radar_ == true)) {
		UpdateRadar(meas_package);
	}
	}

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

	// augment state and covariance
	VectorXd x_aug = VectorXd(n_aug_);
	MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
	MatrixXd Xsig  = MatrixXd(n_aug_, 2 * n_aug_ + 1);
	P_aug.fill(0.0);
	x_aug.fill(0.0);
	
	x_aug.head(n_x_) = x_;
	x_aug(n_x_) = 0;
	x_aug(n_x_ + 1) = 0;

	P_aug.topLeftCorner(n_x_, n_x_) = P_;
	P_aug(n_x_, n_x_) = std_a_ * std_a_;
	P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

	MatrixXd A = P_aug.llt().matrixL();

	// sigma point creation
	Xsig.col(0) = x_aug;
	for (int j = 1; j < 2 * n_aug_ + 1; j++) {
		if (j <= n_aug_) {
			Xsig.col(j) = x_aug + sqrt(lambda_ + n_aug_) * A.col(j-1);
		}
		else
		{
			Xsig.col(j) = x_aug - sqrt(lambda_ + n_aug_) * A.col(j - n_aug_ -1);
		}
	}

	// predict sigma points
	Xsig_pred_ = Xsig.topRows(n_x_);	
	for (int j = 0; j < 2 * n_aug_ + 1; j++) {
		double v = Xsig(2, j);
		double phi = Xsig(3, j);
		double phi_dot = Xsig(4, j);
		double nu_a = Xsig(5, j);
		double nu_phi = Xsig(6, j);

		double delta_t_2 = delta_t * delta_t;
		double phi_dot_x_delta_t = phi_dot *	delta_t;

		VectorXd noise(5);
		noise << 0.5 * delta_t_2 * cos(phi) * nu_a, 0.5*delta_t_2*sin(phi)*nu_a, delta_t * nu_a, 0.5*delta_t_2*nu_phi, delta_t*nu_phi;
		VectorXd change(5);

			if (phi_dot < 0.01) {
				change << v * cos(phi) *delta_t, v*sin(phi)*delta_t, 0, phi_dot_x_delta_t, 0;
			}
			else {
				change << v / phi_dot *(sin(phi + phi_dot*delta_t) - sin(phi)), v / phi_dot*(-cos(phi + phi_dot*delta_t) + cos(phi)), 0, phi_dot_x_delta_t, 0;
			}
			Xsig_pred_.col(j) += change + noise;

	}
	
	// Predict state and covariance
	x_.fill(0.0);
	P_.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		x_ += weights_(i) * Xsig_pred_.col(i);	
	}


	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		while (x_diff(3) > M_PI) {
			x_diff(3) -= 2 * M_PI;
		}
		while (x_diff(3) < -M_PI) {
			x_diff(3) += 2 * M_PI;
		}
		P_ += weights_(i) *	x_diff * x_diff.transpose();
	}

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
	MatrixXd Zsig(2, 2 * n_aug_ + 1);
	VectorXd z_pred(2);
	VectorXd z(2);
	z << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1);
	z_pred.fill(0.0);

	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		Zsig.col(i) << Xsig_pred_(0, i), Xsig_pred_(1, i);
		z_pred += weights_(i) * Zsig.col(i);
	}

	MatrixXd R(2, 2);
	R << pow(std_laspx_,2), 0,
		0, pow(std_laspy_,2);
	
	MatrixXd S(2, 2);
	MatrixXd Tc(n_x_, 2);
	S.fill(0.0);
	Tc.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		VectorXd z_diff = Zsig.col(i) - z_pred;
		while (z_diff(1) > M_PI) {
			z_diff(1) -= 2 * M_PI;
		}
		while (z_diff(1) < -M_PI) {
			z_diff(1) += 2 * M_PI;
		}
		S += weights_(i) * z_diff * z_diff.transpose();

		VectorXd x_diff = Xsig_pred_.col(i) - x_;

		while (x_diff(3) > M_PI) {
			x_diff(3) -= 2 * M_PI;
		}
		while (x_diff(3) < -M_PI) {
			x_diff(3) += 2 * M_PI;
		}
		Tc += weights_(i) * x_diff * z_diff.transpose();
	}
	S += R;

	// Kalman Gain
	MatrixXd K = Tc * S.inverse();

	// Update state
	VectorXd z_diff = z - z_pred;
	while (z_diff(1) > M_PI) {
		z_diff(1) -= 2 * M_PI;
	}
	while (z_diff(1) < -M_PI) {
		z_diff(1) += 2 * M_PI;
	}
	x_ += K * z_diff;

	// Update covariance
	P_ -= K * S * K.transpose();

	// Calculate NIS_laser
	double NIS_laser = z_diff.transpose() * S.inverse() * z_diff;
	cout << "The current measurement is " << z(0) << " and " << z(1) << "\n";
	cout << "My prediction is " << z_pred(0) << " and " << z_pred(1) << "\n";
	cout << "The NIS_laser is " << NIS_laser << "\n";
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  
	MatrixXd Zsig(3, 2 * n_aug_ + 1);
	MatrixXd R(3,3);
	MatrixXd S(3, 3);
	VectorXd z_pred(3);
	MatrixXd Tc(n_x_, 3);

	z_pred.fill(0.0);
	R.fill(0.0);

	// Add variances to R
	R(0, 0) = pow(std_radr_,2);
	R(1, 1) = pow(std_radphi_,2);
	R(2, 2) = pow(std_radrd_,2);

	
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		float px = Xsig_pred_(0,i);
		float py = Xsig_pred_(1,i);
		float v = Xsig_pred_(2,i);
		float theta = Xsig_pred_(3,i);

		float rho = sqrt(px*px + py*py);
		float phi = atan2(py, px);
		float rho_dot = (px * cos(theta) * v + py*sin(theta)*v) / rho;

		Zsig.col(i) << rho, phi, rho_dot;
		z_pred += weights_(i) * Zsig.col(i);
	}
	
	S.fill(0.0);
	Tc.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		VectorXd z_diff = Zsig.col(i) - z_pred;
		while (z_diff(1) > M_PI) {
			z_diff(1) -= 2 * M_PI;
		}
		while (z_diff(1) < -M_PI) {
			z_diff(1) += 2 * M_PI;
		}
		S += weights_(i) * z_diff * z_diff.transpose();

		VectorXd x_diff = Xsig_pred_.col(i) - x_;

		while (x_diff(3) > M_PI) {
			x_diff(3) -= 2 * M_PI;
		}
		while (x_diff(3) < -M_PI) {
			x_diff(3) += 2 * M_PI;
		}
		Tc += weights_(i) * x_diff * z_diff.transpose();
	}
	S += R;
	
	// Kalman gain
	MatrixXd K = Tc * S.inverse();

	// Update state
	VectorXd z(3);
	z << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), meas_package.raw_measurements_(2);

	VectorXd z_diff = z - z_pred;
	while (z_diff(1) > M_PI) {
		z_diff(1) -= 2 * M_PI;
	}
	while (z_diff(1) < -M_PI) {
		z_diff(1) += 2 * M_PI;
	}
	x_ += K * z_diff;

	// Update covariance
	P_ -= K*S*K.transpose();
	
	double NIS_radar = z_diff.transpose() * S.inverse() * z_diff;
	cout << "The current measurement is " << z(0) << " and " << z(1) << " and " << z(2) << "\n";
	cout << "My prediction is " << z_pred(0) << " and " << z_pred(1) << " and " << z_pred(2) << "\n";
	cout << "The current NIS_radar is " << NIS_radar << "\n";
}