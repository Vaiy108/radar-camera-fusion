#pragma once

#include <vector>
#include "detection_types.hpp"
#include "track_types.hpp"

class SimpleTracker {
public:
    SimpleTracker(float maxMatchDistance = 60.0f, int maxMissedFrames = 10);

    std::vector<Track> update(const std::vector<Detection>& detections);

private:
    float distance(const cv::Point2f& a, const cv::Point2f& b) const;

    float maxMatchDistance_;
    int maxMissedFrames_;
    int nextTrackId_;
    std::vector<Track> tracks_;
};