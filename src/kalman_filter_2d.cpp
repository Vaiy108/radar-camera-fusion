#include "kalman_filter_2d.hpp"

// KalmanFilter2D:
// Constant-velocity linear Kalman filter with state:
// [x, y, vx, vy]^T
//
// Measurement model:
// z = [x, y]^T
KalmanFilter2D::KalmanFilter2D()
    : initialized_(false) {
    x_ = cv::Mat::zeros(4, 1, CV_32F);
    P_ = cv::Mat::eye(4, 4, CV_32F);
    F_ = cv::Mat::eye(4, 4, CV_32F);
    Q_ = cv::Mat::eye(4, 4, CV_32F);
    H_ = cv::Mat::zeros(2, 4, CV_32F);
    R_ = cv::Mat::eye(2, 2, CV_32F);
    I_ = cv::Mat::eye(4, 4, CV_32F);

    // Measurement matrix:
    // We directly observe x and y, but not vx or vy.
    H_.at<float>(0, 0) = 1.0f;
    H_.at<float>(1, 1) = 1.0f;

    // Measurement noise covariance.
    // These values control how much the filter trusts the measurement.
    R_.at<float>(0, 0) = 25.0f;
    R_.at<float>(1, 1) = 25.0f;

    // Start with large uncertainty.
    P_ *= 100.0f;
}

// Initialize the filter state using the first position measurement.
void KalmanFilter2D::init(float x, float y) {
    x_.at<float>(0, 0) = x;
    x_.at<float>(1, 0) = y;
    x_.at<float>(2, 0) = 0.0f;
    x_.at<float>(3, 0) = 0.0f;

    P_ = cv::Mat::eye(4, 4, CV_32F) * 100.0f;
    initialized_ = true;
}

// Prediction step:
// Advances the state using a constant-velocity motion model.
void KalmanFilter2D::predict(float dt) {
    if (!initialized_) {
        return;
    }

    // State transition matrix for constant velocity:
    // x_k = x_{k-1} + vx * dt
    // y_k = y_{k-1} + vy * dt
    F_ = cv::Mat::eye(4, 4, CV_32F);
    F_.at<float>(0, 2) = dt;
    F_.at<float>(1, 3) = dt;

    // Simple diagonal process noise.
    // In a more advanced implementation, this would depend on dt and acceleration noise.
    const float q = 1.0f;
    Q_ = cv::Mat::zeros(4, 4, CV_32F);
    Q_.at<float>(0, 0) = q;
    Q_.at<float>(1, 1) = q;
    Q_.at<float>(2, 2) = q;
    Q_.at<float>(3, 3) = q;

    x_ = F_ * x_;
    P_ = F_ * P_ * F_.t() + Q_;
}

// Measurement update step using a 2D position measurement.
void KalmanFilter2D::update(const cv::Point2f& measurement) {
    if (!initialized_) {
        init(measurement.x, measurement.y);
        return;
    }

    cv::Mat z = (cv::Mat_<float>(2, 1) << measurement.x, measurement.y);

    // Innovation / residual
    cv::Mat y = z - H_ * x_;

    // Innovation covariance
    cv::Mat S = H_ * P_ * H_.t() + R_;

    // Kalman gain
    cv::Mat K = P_ * H_.t() * S.inv();

    // Correct state estimate and covariance
    x_ = x_ + K * y;
    P_ = (I_ - K * H_) * P_;
}

// Return filtered position estimate.
cv::Point2f KalmanFilter2D::getPosition() const {
    return cv::Point2f(x_.at<float>(0, 0), x_.at<float>(1, 0));
}

// Return filtered velocity estimate.
cv::Point2f KalmanFilter2D::getVelocity() const {
    return cv::Point2f(x_.at<float>(2, 0), x_.at<float>(3, 0));
}

bool KalmanFilter2D::isInitialized() const {
    return initialized_;
}