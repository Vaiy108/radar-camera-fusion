#include "kalman_filter_2d.hpp"

// KalmanFilter2D:
// State = [x, y, vx, vy]^T
// Camera measurement = [x, y]^T
// Radar measurement = [range, angle]^T (nonlinear EKF update)
KalmanFilter2D::KalmanFilter2D()
    : initialized_(false) {
    x_ = cv::Mat::zeros(4, 1, CV_32F);
    P_ = cv::Mat::eye(4, 4, CV_32F);
    F_ = cv::Mat::eye(4, 4, CV_32F);
    Q_ = cv::Mat::eye(4, 4, CV_32F);
    H_ = cv::Mat::zeros(2, 4, CV_32F);
    R_ = cv::Mat::eye(2, 2, CV_32F);
    I_ = cv::Mat::eye(4, 4, CV_32F);
    R_radar_ = cv::Mat::eye(2, 2, CV_32F);

    // Measurement matrix:
    // Camera observes x and y directly, but not vx or vy.
    H_.at<float>(0, 0) = 1.0f;
    H_.at<float>(1, 1) = 1.0f;

    // Camera measurement noise - Measurement noise covariance.
    // These values control how much the filter trusts the measurement.
    R_.at<float>(0, 0) = 25.0f;
    R_.at<float>(1, 1) = 25.0f;

    // Radar measurement noise: [range, angle]
    R_radar_.at<float>(0, 0) = 16.0f;   // range variance
    R_radar_.at<float>(1, 1) = 0.01f;   // angle variance

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
// Linear camera update using measured image-plane position.
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

// EKF radar update using nonlinear measurement:
// h(x) = [sqrt(px^2 + py^2), atan2(py, px)]
void KalmanFilter2D::updateRadar(float range, float angle) {
    if (!initialized_) {
        // Convert polar to Cartesian for first initialization
        float px = range * std::cos(angle);
        float py = range * std::sin(angle);
        init(px, py);
        return;
    }

    float px = x_.at<float>(0, 0);
    float py = x_.at<float>(1, 0);

    float c1 = px * px + py * py;
    if (c1 < 1e-4f) {
        c1 = 1e-4f;
    }

    float sqrt_c1 = std::sqrt(c1);

    // Predicted nonlinear radar measurement h(x)
    cv::Mat h = (cv::Mat_<float>(2, 1) <<
        sqrt_c1,
        std::atan2(py, px));

    // Actual radar measurement z
    cv::Mat z = (cv::Mat_<float>(2, 1) << range, angle);

    // Innovation
    cv::Mat y = z - h;
    y.at<float>(1, 0) = normalizeAngle(y.at<float>(1, 0));

    // Jacobian Hj = dh/dx
    cv::Mat Hj = cv::Mat::zeros(2, 4, CV_32F);

    Hj.at<float>(0, 0) = px / sqrt_c1;
    Hj.at<float>(0, 1) = py / sqrt_c1;

    Hj.at<float>(1, 0) = -py / c1;
    Hj.at<float>(1, 1) = px / c1;

    cv::Mat S = Hj * P_ * Hj.t() + R_radar_;
    cv::Mat K = P_ * Hj.t() * S.inv();

    x_ = x_ + K * y;
    P_ = (I_ - K * Hj) * P_;
}

float KalmanFilter2D::normalizeAngle(float angle) const {
    while (angle > static_cast<float>(CV_PI)) {
        angle -= 2.0f * static_cast<float>(CV_PI);
    }
    while (angle < -static_cast<float>(CV_PI)) {
        angle += 2.0f * static_cast<float>(CV_PI);
    }
    return angle;
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