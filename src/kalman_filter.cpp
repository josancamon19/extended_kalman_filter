#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

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

void KalmanFilter::Predict() {
    /**
     * TODO: predict the state
     */
    // cout << "F size: " << F_.size() << ", x_ size: " << x_.size() << ", P_.size: " << P_.size() << ", Q size: "
    //      << Q_.size() << "\n";
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
    /**
     * TODO: update the state by using Kalman Filter equations
     */

    VectorXd y = z - H_ * x_;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd K = P_ * Ht * S.inverse();

    x_ = x_ + K * y;
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    // cout << "I:" << I << "\n\n";
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
     * TODO: update the state by using Extended Kalman Filter equations
     */
    // cout << "EKF z: " << z;

    double px = x_(0);
    double py = x_(1);
    double vx = x_(2);
    double vy = x_(3);

    double eq1 = sqrt(px * px + py * py);
    double eq2 = atan(py / px);
    double eq3 = (px * vx + py * vy) / eq1;

    VectorXd h_x(3);
    h_x << eq1, eq2, eq3;

    VectorXd y = z - h_x;

    while (y(1) > M_PI) {
        y(1) -= 2 * M_PI;
    }
    while (y(1) < -M_PI) {
        y(1) += 2 * M_PI;
    }


    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd K = P_ * Ht * S.inverse();

    x_ = x_ + K * y;
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    // cout << "I:" << I << "\n\n";
    P_ = (I - K * H_) * P_;

}
