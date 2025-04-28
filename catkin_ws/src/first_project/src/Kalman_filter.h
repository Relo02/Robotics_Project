#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter(const Eigen::VectorXd& initial_state,
                const Eigen::MatrixXd& initial_covariance,
                const Eigen::MatrixXd& process_noise,
                const Eigen::MatrixXd& gps_noise);

    void predict(double dt, double v, double omega);
    void update(const Eigen::Vector2d& gps_measurement);

    Eigen::VectorXd getState() const;
    Eigen::Vector2d getPosition() const;

private:
    Eigen::VectorXd state_;        // 5x1
    Eigen::MatrixXd covariance_;   // 5x5
    Eigen::MatrixXd Q_;            // 5x5
    Eigen::MatrixXd R_;            // 2x2
    Eigen::MatrixXd F_;            // 5x5
    Eigen::MatrixXd H_;            // 2x5
    Eigen::VectorXd get_state();
    double d_;                     // Wheelbase
};

#endif // KALMANFILTER_H
