// Associates camera tracks with radar detections and computes fused positions.

#pragma once

#include <vector>
#include "track_types.hpp"
#include "radar_types.hpp"
#include "fusion_types.hpp"

class FusionManager {
public:
    FusionManager(float maxAssociationDistance = 50.0f,
                  float cameraWeight = 0.6f,
                  float radarWeight = 0.4f);

    std::vector<FusedTrack> fuse(const std::vector<Track>& cameraTracks,
                                 const std::vector<RadarDetection>& radarDetections);

private:
    float distance(const cv::Point2f& a, const cv::Point2f& b) const;

    float maxAssociationDistance_;
    float cameraWeight_;
    float radarWeight_;
};