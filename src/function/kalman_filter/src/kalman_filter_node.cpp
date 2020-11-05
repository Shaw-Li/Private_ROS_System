#include <ros/ros.h>

#include <iostream>

#include "kalman_filter.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "kalman_filter");
    double m_x = 0.0, m_y = 0.0;
    double last_timestamp = 0.0, now_timestamp = 0.0;
    KalmanFilter kf_object;

    while (ros::ok) {
        if (!kf_object.IsInitialized()) {
            last_timestamp = now_timestamp;
            Eigen::VectorXd x_in(4, 1);
            Eigen::VectorXd u_in(4, 1);
            x_in << m_x, m_y, 0.0, 0.0;
            u_in << 0.0, 0.0, 0.0, 0.0;
            kf_object.Initialization(x_in, u_in);
            Eigen::MatrixXd P_in(4, 4);
            P_in << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 100.0;
            kf_object.SetP(P_in);
            Eigen::MatrixXd Q_in(4, 4);
            Q_in << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
            kf_object.SetQ(Q_in);
            Eigen::MatrixXd H_in(2, 4);
            H_in << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
            kf_object.SetH(H_in);
            Eigen::MatrixXd R_in(2, 4);
            R_in << 0.0225, 0.0, 0.0, 0.0225;
            kf_object.SetR(R_in);
        }
        double delta_t = now_timestamp - last_timestamp;
        last_timestamp = now_timestamp;
        Eigen::MatrixXd F_in(4, 4);
        F_in << 1.0, 0.0, delta_t, 0.0, 0.0, 1.0, 0.0, delta_t, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
        kf_object.SetF(F_in);
        kf_object.Prediction();
        Eigen::VectorXd z(2, 1);
        z << m_x, m_y;
        kf_object.MeasurementUpdate(z);
        Eigen::VectorXd x_out = kf_object.GetX();
        std::cout << "kalman output x : " << x_out(0) << " y : " << x_out(1) << std::endl;
    }
    return 0;
}