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

    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

    /**
     * TODO: Finish initializing the FusionEKF.
     * TODO: Set the process and measurement noises
     */
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

VectorXd polarToCart(VectorXd x) {
    VectorXd cart_coord(4);
    // cout << "x: " << x << endl;
    double rho = x(0);
    double phi = x(1);
    double rho_dot = x(2);

    cart_coord(0) = cos(phi) * rho;
    cart_coord(1) = sin(phi) * rho;
    cart_coord(2) = cos(phi) * rho_dot;
    cart_coord(3) = sin(phi) * rho_dot;

    return cart_coord;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    /**
     * Initialization
     */
    if (!is_initialized_) {
        /** DONE
         * TODO: Initialize the state ekf_.x_ with the first measurement.
         * TODO: Create the covariance matrix.
         * You'll need to convert radar from polar to cartesian coordinates.
         */

        // first measurement
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;

        ekf_.P_ = MatrixXd(4, 4);
        ekf_.P_ << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;

        // F_ wont be used yet, but defining F in the initialization saves time
        ekf_.F_ = MatrixXd(4, 4);
        ekf_.F_ << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

        cout << "Initializing" << endl;
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            // TODO: Convert radar from polar to cartesian coordinates
            //         and initialize state.

            cout << "RADAR: " << measurement_pack.raw_measurements_ << "\n";
            ekf_.x_ = polarToCart(measurement_pack.raw_measurements_);
        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            // TODO: Initialize state.
            // cout << "LASER: " << measurement_pack.raw_measurements_ << "\n";
            ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 0, 0;

        }
        previous_timestamp_ = measurement_pack.timestamp_;
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /**
     * Prediction
     */
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;

    double noise_ax = 9;
    double noise_ay = 9;

    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << (dt4 / 4) * noise_ax, 0, (dt3 / 2) * noise_ax, 0,
            0, (dt4 / 4) * noise_ay, 0, (dt3 / 2) * noise_ay,
            (dt3 / 2) * noise_ax, 0, dt2 * noise_ax, 0,
            0, (dt3 / 2) * noise_ay, 0, dt2 * noise_ay;
    /**
     * TODO: Update the state transition matrix F according to the new elapsed time.
     * Time is measured in seconds.
     * TODO: Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */

    ekf_.Predict();


    /**
     * Update
     */

    /**
     * TODO:
     * - Use the sensor type to perform the update step.
     * - Update the state and covariance matrices.
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // TODO: Radar updates
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.R_ = R_radar_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
        // TODO: Laser updates
        // cout << "\n" << ekf_.H_ << endl;
        // cout << "\n" << H_laser_ << endl;
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;

        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
    // cout << "x_ = " << ekf_.x_ << endl;
    // cout << "P_ = " << ekf_.P_ << endl;
}
