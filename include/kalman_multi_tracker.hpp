#pragma once

#include <vector>
#include "detection_types.hpp"
#include "kalman_track.hpp"
#include "radar_types.hpp"

class KalmanMultiTracker {
public:
    KalmanMultiTracker(float maxMatchDistance = 120.0f, int maxMissedFrames = 15);

    std::vector<KalmanTrack> update(const std::vector<Detection>& detections, float dt);

    // EKF radar correction step after camera tracking
    void updateWithRadar(const std::vector<RadarDetection>& radarDetections);

    std::vector<KalmanTrack> getTracks() const;

private:
    float distance(const cv::Point2f& a, const cv::Point2f& b) const;

    float maxMatchDistance_;
    int maxMissedFrames_;
    int nextTrackId_;
    std::vector<KalmanTrack> tracks_;
};