#include "tools.h"
#include <iostream>

using namespace std;

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    /**
     * TODO: Calculate the RMSE here.
     */

    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    if (estimations.size() != ground_truth.size()) {
        cout << "Error in RMSE, estimations.size != ground_t.size";
        return rmse;
    }

    for (int i = 0; i < estimations.size(); ++i) {
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse += residual;
    }
    rmse = rmse / estimations.size();
    // rmse = rmse.array().sqrt();
    rmse = sqrt(rmse.array());
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
    /**
     * TODO:
     * Calculate a Jacobian here.
     */
    MatrixXd Hj(3, 4);

    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);

    double d = px * px + py * py;

    if (fabs(d) < 0.00001) {
        // cout << "Error calculating Jacobian, division by 0.";
        px =  0.001;
        py = 0.001;

        d = px*px + py*py;
    }

    double sqr = sqrt(d);
    double tt = pow(d, 1.5);

    Hj << px / sqr, py / sqr, 0, 0,
            -py / d, px / d, 0, 0,
            py * (vx * py - vy * px) / tt, px * (vy * px - vx * py) / tt, px / sqr, py / sqr;

}
