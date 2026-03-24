#pragma once

#include <opencv2/core.hpp>

class KalmanFilter2D {
public:
    KalmanFilter2D();

    void init(float x, float y);

    void predict(float dt);

    // Linear camera update: measurement = [x, y]
    void update(const cv::Point2f& measurement);

    // Nonlinear radar EKF update: measurement = [range, angle]
    void updateRadar(float range, float angle);

    cv::Point2f getPosition() const;
    cv::Point2f getVelocity() const;
    bool isInitialized() const;

private:
    float normalizeAngle(float angle) const;

    bool initialized_;

    // State: [x, y, vx, vy]^T
    cv::Mat x_; // 4x1
    cv::Mat P_; // 4x4
    cv::Mat F_; // 4x4
    cv::Mat Q_; // 4x4
    cv::Mat H_; // 2x4
    cv::Mat R_; // 2x2 camera measurement noise
    cv::Mat I_; // 4x4

    // Radar measurement noise
    cv::Mat R_radar_; // 2x2
};