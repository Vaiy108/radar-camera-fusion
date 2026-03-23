#include "radar_simulator.hpp"
#include <cmath>

RadarSimulator::RadarSimulator(float positionNoiseStd)
    : rng_(std::random_device{}()),
      positionNoiseStd_(positionNoiseStd) {
}

float RadarSimulator::noise(float stddev) {
    std::normal_distribution<float> dist(0.0f, stddev);
    return dist(rng_);
}

std::vector<RadarDetection> RadarSimulator::simulate(const std::vector<Track>& tracks) {
    std::vector<RadarDetection> radarDetections;
    radarDetections.reserve(tracks.size());

    for (const auto& track : tracks) {
        cv::Point2f noisyPosition(
            track.center.x + noise(positionNoiseStd_),
            track.center.y + noise(positionNoiseStd_)
        );

        float range = std::sqrt(noisyPosition.x * noisyPosition.x +
                                noisyPosition.y * noisyPosition.y);

        float angle = std::atan2(noisyPosition.y, noisyPosition.x);

        RadarDetection rd;
        rd.associatedTrackId = track.id;
        rd.position = noisyPosition;
        rd.range = range;
        rd.angle = angle;

        radarDetections.push_back(rd);
    }

    return radarDetections;
}