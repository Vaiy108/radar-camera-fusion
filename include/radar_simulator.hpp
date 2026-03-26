// Generates noisy synthetic radar detections from tracked object positions.

#pragma once

#include <vector>
#include <random>
#include "track_types.hpp"
#include "radar_types.hpp"

class RadarSimulator {
public:
    RadarSimulator(float positionNoiseStd = 4.0f);

    std::vector<RadarDetection> simulate(const std::vector<Track>& tracks);

private:
    float noise(float stddev);

    std::mt19937 rng_;
    float positionNoiseStd_;
};