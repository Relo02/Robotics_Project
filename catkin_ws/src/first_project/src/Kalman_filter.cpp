#include "src/Kalman_filter.h"
#include <cmath>

KalmanFilter::KalmanFilter(const Eigen::VectorXd& initial_state,
                           const Eigen::MatrixXd& initial_covariance,
                           const Eigen::MatrixXd& process_noise,
                           const Eigen::MatrixXd& gps_noise)
    : state_(initial_state),
      covariance_(initial_covariance),
      Q_(process_noise),
      R_(gps_noise),
      d_(1.765)  // Wheelbase
{
    F_ = Eigen::MatrixXd::Identity(5, 5);
    H_.resize(2, 5);
    H_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0;
}

void KalmanFilter::predict(double dt, double v, double omega) {
    double theta = state_(2);

    if (std::abs(omega) < 1e-6) {
        state_(0) += v * dt * std::sin(theta + (omega * dt) / 2.0);
        state_(1) += v * dt * std::cos(theta + (omega * dt) / 2.0);
        state_(2) += omega * dt;
    } else {
        state_(0) += (v / omega) * (std::cos(theta) - std::cos(theta + omega * dt));
        state_(1) += (v / omega) * (std::sin(theta + omega * dt) - std::sin(theta));
        state_(2) += omega * dt;
    }

    state_(3) = v;
    state_(4) = omega;

    if (std::abs(omega) < 1e-6) {
        F_ << 1, 0, v * dt * std::cos(theta), dt * std::sin(theta), 0,
              0, 1, -v * dt * std::sin(theta), dt * std::cos(theta), 0,
              0, 0, 1, 0, dt,
              0, 0, 0, 1, 0,
              0, 0, 0, 0, 1;
    } else {
        double cth = std::cos(theta);
        double sth = std::sin(theta);
        double cthw = std::cos(theta + omega * dt);
        double sthw = std::sin(theta + omega * dt);

        F_ << 1, 0, (v / omega) * (sth - sthw), (1 / omega) * (cth - cthw), (v / (omega * omega)) * (cthw - cth) + (v * dt / omega) * sthw,
              0, 1, (v / omega) * (cthw - cth), (1 / omega) * (sthw - sth), (v / (omega * omega)) * (sthw - sth) - (v * dt / omega) * cthw,
              0, 0, 1, 0, dt,
              0, 0, 0, 1, 0,
              0, 0, 0, 0, 1;
    }

    covariance_ = F_ * covariance_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::Vector2d& gps_measurement) {
    Eigen::Vector2d y = gps_measurement - H_ * state_;
    Eigen::Matrix2d S = H_ * covariance_ * H_.transpose() + R_;
    Eigen::MatrixXd K = covariance_ * H_.transpose() * S.inverse();

    state_ += K * y;

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(covariance_.rows(), covariance_.cols());
    covariance_ = (I - K * H_) * covariance_;
}

Eigen::VectorXd KalmanFilter::getState() const {
    return state_;
}

Eigen::Vector2d KalmanFilter::getPosition() const {
    return state_.head<2>();
}
