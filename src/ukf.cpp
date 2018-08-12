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
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // Initialization status
  is_initialized_ = false;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Predicted sigma points matrix
  Xsig_pred_ = MatrixXd(5, 15);

  // Time when state is true
  time_us_ = 0;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.2;

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

  // Add dimensions of state vector
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // Initialize weight vector for augmented state
  weights_ = VectorXd(2*n_aug_ + 1);
  weights_.fill(0.5/(lambda_ + n_aug_));
  weights_(0) *= 2*lambda_;

  // Initialize NIS values
  NIS_radar_ = 0.;
  NIS_lidar_ = 0.;
}

UKF::~UKF() {}

/**
 * Initializes the state and and the state covariance matrix using measurement
 * data.
 * @param {MeasurementPackage} meas_package
 */
 void UKF::x_P_Initialization(MeasurementPackage meas_package) {
  // first measurement
  cout << "UKF: " << endl;
  x_ = VectorXd(5);
  P_ = MatrixXd(5,5);

  // Remember timestamp of first measurements
  time_us_ = meas_package.timestamp_;

  // Initialize state-vector x with first measurement
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    /**
    Convert radar from polar to cartesian coordinates and initialize state.
    **/
    double rho = meas_package.raw_measurements_(0);
    double phi = meas_package.raw_measurements_(1);
    x_(0) = rho     * cos(phi);
    x_(1) = rho     * sin(phi);

    P_.fill(0.0);
    P_(0,0) = std_radr_*std_radr_;
    P_(1,1) = std_radr_*std_radr_;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    /**
    Initialize state.
    */
    x_(0) = meas_package.raw_measurements_(0);
    x_(1) = meas_package.raw_measurements_(1);

    P_.fill(0.0);
    P_(0,0) = std_laspx_*std_laspx_;
    P_(1,1) = std_laspy_*std_laspy_;
  }
  // Tuning parameters state vector
  x_(2) = 5.;  // v can be tuned
  x_(3) = 0.;  // yaw can be tuned
  x_(4) = 0.;  // yaw rate can be tuned
  // Tuning parameters covariance matrix
  P_(2,2) = 5;  // v uncertainty
  P_(3,3) = 10; // yaw uncertainty
  P_(4,4) = 5;  // yaw rate uncertainty
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;

  // done initializing, no need to predict or update
  is_initialized_ = true;
  return;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  /*****************************************************************************
  *  Initialization
  *****************************************************************************/
  if (!is_initialized_) {
    x_P_Initialization(meas_package);
    // print the output
    cout << "x_ init = " << endl << x_ << endl;
    cout << "P_ init = " << endl << P_ << endl;
    return;
  }

  /*****************************************************************************
  *  Prediction
  *****************************************************************************/
  // Call prediction
  double delta_t = (meas_package.timestamp_ - time_us_) * 0.000001;
  time_us_ = meas_package.timestamp_;
  Prediction(delta_t);

  /*****************************************************************************
  *  Update
  *****************************************************************************/
  // Call update
  if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) & use_radar_) {
    UpdateRadar(meas_package);
  } else if ((meas_package.sensor_type_ == MeasurementPackage::LASER) & use_laser_) {
    UpdateLidar(meas_package);
  } else {
    cout << "Predicted values used instead of measurement" << endl;
  }
  // print the output
  cout << "x_ = " << endl << x_ << endl;
  cout << "P_ = " << endl << P_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /** This part of the code was discarded for speed up purposes.
  It is not necessary for any calculations downstream.
  // GENERATE SIGMA POINTS
  lambda_ = 3 - n_x_;  // Define spreading parameter lambda

  //calculate square root matrix A
  MatrixXd A = P_.llt().matrixL();

  MatrixXd Xsig = MatrixXd(n_x_,2*n_x_+1);
  Xsig.col(0) = x_;
  for (int j = 0; j < n_x_; j++) {
    Xsig.col(j+1)         = x_ + A.col(j)*sqrt(lambda_ + n_x_);
    Xsig.col(j+1+n_x_)    = x_ - A.col(j)*sqrt(lambda_ + n_x_);
  }
  **/

  // GENERATE AUGMENTED SIGMA POINTS (INCLUDING PROCESS NOISE)
  lambda_ = 3 - n_aug_;
  // 1st: Create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(n_x_) = x_;
  x_aug(5) = 0.; // mean process noise = 0.0
  x_aug(6) = 0.; // mean process noise = 0.0
  // 2nd: Create augmented state covariance matrix
  MatrixXd P_aug = MatrixXd(n_aug_,n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;
  // 3rd: Create square root MatrixXd
  MatrixXd A_aug = P_aug.llt().matrixL();
  // 4th: Create augmented sigma points
  MatrixXd Xsig_aug = MatrixXd(7,15);
  Xsig_aug.col(0) = x_aug;
  for (int j = 0; j < n_aug_; j++) {
    Xsig_aug.col(j+1)         = x_aug + A_aug.col(j)*sqrt(lambda_ + n_aug_);
    Xsig_aug.col(j+1+n_aug_)  = x_aug - A_aug.col(j)*sqrt(lambda_ + n_aug_);
  }

  // PREDICT SIGMA POINTS
  // 1st: Allocate variable & initialize with zeros
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);
  Xsig_pred_.fill(0.0);
  // 2nd: Walk over all columns
  for (int i=0; i < 2*n_aug_+1; i++) {
    // 3rd: extract values for better readability
    double px       = Xsig_aug(0,i);
    double py       = Xsig_aug(1,i);
    double v        = Xsig_aug(2,i);
    double yaw      = Xsig_aug(3,i);
    double yawd     = Xsig_aug(4,i);
    double nu_a     = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);
    // 4th: predict state values & avoid division by zero
    if (abs(yawd) <= 0.001) {
        Xsig_pred_(0,i) = px + v*cos(yaw)*delta_t;
        Xsig_pred_(1,i) = py + v*sin(yaw)*delta_t;
    } else {
        Xsig_pred_(0,i) = px + v/yawd * ( sin(yaw+yawd*delta_t)-sin(yaw));
        Xsig_pred_(1,i) = py + v/yawd * (-cos(yaw+yawd*delta_t)+cos(yaw));
    }
    Xsig_pred_(2,i) = v;
    Xsig_pred_(3,i) = yaw + yawd*delta_t;
    Xsig_pred_(4,i) = yawd;
    // 5th: add noise
    Xsig_pred_(0,i) += 0.5*delta_t * delta_t * cos(yaw) * nu_a;
    Xsig_pred_(1,i) += 0.5*delta_t * delta_t * sin(yaw) * nu_a;
    Xsig_pred_(2,i) +=     delta_t * nu_a;
    Xsig_pred_(3,i) += 0.5*delta_t * delta_t * nu_yawdd;
    Xsig_pred_(4,i) +=     delta_t * nu_yawdd;
  }

  // PREDICT STATE & COVARIANCE MATRIX
  // 1st: predict state vector
  VectorXd x_pred = VectorXd(n_x_);
  x_pred.fill(0.0);
  for (int i=0; i<n_x_; i++) {
    //predict state mean
    x_pred(i) += Xsig_pred_.row(i) * weights_;
  }
  // 2nd: predict state covariance matrix
  MatrixXd P_pred = MatrixXd(n_x_,n_x_);
  P_pred.fill(0.0);
  VectorXd x_diff = VectorXd(n_x_);
  x_diff.fill(0.0);

  for (int i=0; i<2*n_aug_+1; i++) {
    // State difference
    x_diff = Xsig_pred_.col(i) - x_pred;
    // Angle normalization
    while (x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3) <-M_PI) x_diff(3) += 2.*M_PI;
    // Predict state covariance matrix
    P_pred += weights_(i) * (x_diff * x_diff.transpose());
  }
  // VectorXd x_pred_ = VectorXd(n_x_);
  x_ = x_pred;
  P_ = P_pred;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  // TRANSFORM PREDICTION TO LIDAR MEASUREMENT SPACE
  // 1st: create matrix for sigma points in measurement space
  int n_z = 2;
  MatrixXd Zsig_pred = MatrixXd(n_z, 2* n_aug_ + 1);
  // 2nd: loop over all columns to fill up Zsig
  for (int i = 0; i < 2*n_aug_+1; i++) {
    // Read variables from current colum for better readability
    double px   = Xsig_pred_(0,i);
    double py   = Xsig_pred_(1,i);

    // Write back to Zsig
    Zsig_pred.col(i) <<   px,
                          py;
  }
  // 3rd: Predict vector z
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < n_z; i++) {
    z_pred.row(i) = Zsig_pred.row(i) * weights_;
  }
  // 4th: Calculate innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
    VectorXd z_diff = Zsig_pred.col(i) - z_pred;
    S += weights_(i) * z_diff * z_diff.transpose();
  }
  // 5th: Calculate measurement noise covariance matrix R
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<  std_laspx_*std_laspx_,  0,
        0,                      std_laspy_*std_laspy_;
  S += R;


  // UPDATE LIDAR STATE & COVARIANCE
  // 1st: Initialize necessary varialbes
  MatrixXd Tc = MatrixXd(n_x_,n_z);
  MatrixXd X_diff = MatrixXd(n_x_, 2*n_aug_+1);
  MatrixXd Z_diff = MatrixXd(n_z,  2*n_aug_+1);
  // 2nd: Prepare matrices before Tc calculation can take place.
  for (int i = 0; i < 2*n_aug_+1; i++) {
    X_diff.col(i) = Xsig_pred_.col(i) - x_;
    while (X_diff(3,i) >  M_PI) X_diff(3,i) -= 2.*M_PI;
    while (X_diff(3,i) < -M_PI) X_diff(3,i) += 2.*M_PI;
    Z_diff.col(i) = Zsig_pred.col(i) - z_pred;
  }

  // 3rd: Calculate matrix for cross-correlation Tc, correct angles
  MatrixXd Z_diff_t = Z_diff.transpose();
  Tc.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
    Tc += weights_(i) * (X_diff.col(i) * Z_diff_t.row(i));
  }
  // 4th: Calculate Kalman gain K
  MatrixXd K = Tc * S.inverse();

  // 5th: Finally update state mean and covariance matrix
  VectorXd z = meas_package.raw_measurements_; // read in from measurement
  VectorXd z_diff = z - z_pred;
  x_ +=  K * z_diff;
  P_ += -K * S * K.transpose();

  // CALCULATE LIDAR NIS
  NIS_lidar_ = z_diff.transpose() * S.inverse() * z_diff;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  // TRANSFORM PREDICTION TO RADAR MEASUREMENT SPACE
  // 1st: create matrix for sigma points in measurement space
  int n_z = 3;
  MatrixXd Zsig_pred = MatrixXd(n_z, 2 * n_aug_ + 1);
  // 2nd: loop over all columns to fill up Zsig
  for (int i = 0; i < 2*n_aug_+1; i++) {
    // Read variables from current colum for better readability
    double px   = Xsig_pred_(0,i);
    double py   = Xsig_pred_(1,i);
    double v    = Xsig_pred_(2,i);
    double yaw  = Xsig_pred_(3,i);
    // double yawd = Xsig_pred_(4,i);
    // Calculate rho
    double rho = sqrt(px*px + py*py);
    // Calculate phi including angle correction to -PI...+PI
    double phi = atan2(py,px);
    while (phi < -M_PI) phi += 2.*M_PI;
    while (phi >  M_PI) phi -= 2.*M_PI;
    // Calculate rho dot
    double rhod = v*(px*cos(yaw) + py*sin(yaw)) / rho;
    // Write back to Zsig
    Zsig_pred.col(i) <<   rho,
                          phi,
                          rhod;
  }
  // 3rd: Predict vector z
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < n_z; i++) {
    z_pred.row(i) = Zsig_pred.row(i) * weights_;
  }
  // 4th: Calculate innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
    VectorXd z_diff = Zsig_pred.col(i) - z_pred;
    while (z_diff(1) >  M_PI) z_diff(1) -= 2.*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;
    S += weights_(i) * z_diff * z_diff.transpose();
  }
  // 5th: Calculate measurement noise covariance matrix R
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<  std_radr_*std_radr_,  0,                        0,
        0,                    std_radphi_*std_radphi_,  0,
        0,                    0,                        std_radrd_*std_radrd_;
  S += R;


  // UPDATE RADAR STATE & COVARIANCE
  // 1st: Initialize necessary varialbes
  MatrixXd Tc = MatrixXd(n_x_,n_z);
  MatrixXd X_diff = MatrixXd(n_x_, 2*n_aug_+1);
  MatrixXd Z_diff = MatrixXd(n_z,  2*n_aug_+1);
  // 2nd: Prepare matrices before Tc calculation can take plase. Angle correction
  for (int i = 0; i < 2*n_aug_+1; i++) {
    X_diff.col(i) = Xsig_pred_.col(i) - x_;
    while (X_diff(3,i) >  M_PI) X_diff(3,i) -= 2.*M_PI;
    while (X_diff(3,i) < -M_PI) X_diff(3,i) += 2.*M_PI;
    Z_diff.col(i) = Zsig_pred.col(i) - z_pred;
    while (Z_diff(1,i) >  M_PI) Z_diff(1,i) -= 2.*M_PI;
    while (Z_diff(1,i) < -M_PI) Z_diff(1,i) += 2.*M_PI;
  }

  // 3rd: Calculate matrix for cross-correlation Tc, correct angles
  MatrixXd Z_diff_t = Z_diff.transpose();
  Tc.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
    Tc += weights_(i) * (X_diff.col(i) * Z_diff_t.row(i));
  }
  // 4th: Calculate Kalman gain K
  MatrixXd K = Tc * S.inverse();

  // 5th: Finally update state mean and covariance matrix
  VectorXd z = meas_package.raw_measurements_; // read in from measurement
  VectorXd z_diff = z - z_pred;
  while (z_diff(1) >  M_PI) z_diff(1) -= 2.*M_PI;
  while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;
  x_ +=  K * z_diff;
  P_ += -K * S * K.transpose();

  // CALCULATE RADAR NIS
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}
