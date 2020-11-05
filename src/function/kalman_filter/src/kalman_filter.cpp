#include "kalman_filter.h"

KalmanFilter::KalmanFilter() { is_initialized = false; }

KalmanFilter::~KalmanFilter() {}

VectorXd KalmanFilter::GetX() { return x_; }

bool KalmanFilter::IsInitialized() { return is_initialized; }

void KalmanFilter::Initialization(VectorXd x_in, VectorXd u_in) {
    x_ = x_in;
    u_ = u_in;
    is_initialized = true;
}

void KalmanFilter::SetF(MatrixXd F_in) { F_ = F_in; }

void KalmanFilter::SetP(MatrixXd P_in) { P_ = P_in; }

void KalmanFilter::SetQ(MatrixXd Q_in) { Q_ = Q_in; }

void KalmanFilter::SetH(MatrixXd H_in) { H_ = H_in; }

void KalmanFilter::SetR(MatrixXd R_in) { R_ = R_in; }

void KalmanFilter::Prediction() {
    x_pre_ = F_ * x_ + u_;  //+u
    MatrixXd Ft = F_.transpose();
    P_pre_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::MeasurementUpdate(const VectorXd &z) {
    VectorXd y = z - H_ * x_pre_;
    MatrixXd S = H_ * P_pre_ * H_.transpose() + R_;
    MatrixXd K = P_pre_ * H_.transpose() * S.inverse();
    x_ = x_pre_ + K * y;
    int size = x_.size();
    MatrixXd I = I.Identity(size, size);
    P_ = (I - K * H_) * P_pre_;
}
