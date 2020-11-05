#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
#include <Eigen/Dense>

using namespace Eigen;

class KalmanFilter {
   private:
    bool is_initialized;
    VectorXd x_pre_;
    MatrixXd P_pre_;
    VectorXd x_;
    VectorXd u_;
    MatrixXd F_;
    MatrixXd P_;
    MatrixXd Q_;
    MatrixXd H_;
    MatrixXd R_;

   public:
    KalmanFilter();
    ~KalmanFilter();

    VectorXd GetX();
    bool IsInitialized();
    void Initialization(VectorXd x_in, VectorXd u_in);
    void SetF(MatrixXd F_in);
    void SetP(MatrixXd P_in);
    void SetQ(MatrixXd Q_in);
    void SetH(MatrixXd H_in);
    void SetR(MatrixXd R_in);
    void Prediction();
    void MeasurementUpdate(const VectorXd &z);
};
#endif