#include "fusion_manager.hpp"
#include <cmath>
#include <limits>

FusionManager::FusionManager(float maxAssociationDistance,
                             float cameraWeight,
                             float radarWeight)
    : maxAssociationDistance_(maxAssociationDistance),
      cameraWeight_(cameraWeight),
      radarWeight_(radarWeight) {
}

float FusionManager::distance(const cv::Point2f& a, const cv::Point2f& b) const {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<FusedTrack> FusionManager::fuse(const std::vector<Track>& cameraTracks,
                                            const std::vector<RadarDetection>& radarDetections) {
    std::vector<FusedTrack> fusedTracks;
    std::vector<bool> radarUsed(radarDetections.size(), false);

    for (const auto& camTrack : cameraTracks) {
        float bestDistance = std::numeric_limits<float>::max();
        int bestRadarIndex = -1;

        for (size_t i = 0; i < radarDetections.size(); ++i) {
            if (radarUsed[i]) {
                continue;
            }

            float dist = distance(camTrack.center, radarDetections[i].position);
            if (dist < bestDistance && dist < maxAssociationDistance_) {
                bestDistance = dist;
                bestRadarIndex = static_cast<int>(i);
            }
        }

        FusedTrack fused;
        fused.id = camTrack.id;
        fused.cameraPosition = camTrack.center;
        fused.hasCamera = true;

        if (bestRadarIndex >= 0) {
            radarUsed[bestRadarIndex] = true;

            fused.radarPosition = radarDetections[bestRadarIndex].position;
            fused.hasRadar = true;

            fused.fusedPosition.x =
                cameraWeight_ * fused.cameraPosition.x +
                radarWeight_ * fused.radarPosition.x;

            fused.fusedPosition.y =
                cameraWeight_ * fused.cameraPosition.y +
                radarWeight_ * fused.radarPosition.y;
        } else {
            fused.fusedPosition = fused.cameraPosition;
        }

        fusedTracks.push_back(fused);
    }

    return fusedTracks;
}