#include "kalman_multi_tracker.hpp"
#include <cmath>
#include <limits>

KalmanMultiTracker::KalmanMultiTracker(float maxMatchDistance, int maxMissedFrames)
    : maxMatchDistance_(maxMatchDistance),
      maxMissedFrames_(maxMissedFrames),
      nextTrackId_(0) {
}

float KalmanMultiTracker::distance(const cv::Point2f& a, const cv::Point2f& b) const {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<KalmanTrack> KalmanMultiTracker::update(const std::vector<Detection>& detections, float dt) {
    // Predict all tracks first
    for (auto& track : tracks_) {
        track.kf.predict(dt);
        track.filteredCenter = track.kf.getPosition();
        track.velocity = track.kf.getVelocity();
    }

    std::vector<bool> detectionMatched(detections.size(), false);
    std::vector<bool> trackMatched(tracks_.size(), false);

    // Greedy nearest-neighbor matching using predicted positions
    for (size_t t = 0; t < tracks_.size(); ++t) {
        float bestDistance = std::numeric_limits<float>::max();
        int bestDetectionIndex = -1;

        for (size_t d = 0; d < detections.size(); ++d) {
            if (detectionMatched[d]) {
                continue;
            }

            float dist = distance(tracks_[t].filteredCenter, detections[d].center);
            if (dist < bestDistance && dist < maxMatchDistance_) {
                bestDistance = dist;
                bestDetectionIndex = static_cast<int>(d);
            }
        }

        if (bestDetectionIndex >= 0) {
            const auto& det = detections[bestDetectionIndex];

            tracks_[t].bbox = det.bbox;
            tracks_[t].measuredCenter = det.center;
            tracks_[t].kf.update(det.center);
            tracks_[t].filteredCenter = tracks_[t].kf.getPosition();
            tracks_[t].velocity = tracks_[t].kf.getVelocity();
            tracks_[t].age += 1;
            tracks_[t].missedFrames = 0;

            detectionMatched[bestDetectionIndex] = true;
            trackMatched[t] = true;
        }
    }

    // Unmatched tracks
    for (size_t t = 0; t < tracks_.size(); ++t) {
        if (!trackMatched[t]) {
            tracks_[t].age += 1;
            tracks_[t].missedFrames += 1;
            tracks_[t].filteredCenter = tracks_[t].kf.getPosition();
            tracks_[t].velocity = tracks_[t].kf.getVelocity();
        }
    }

    // New tracks
    for (size_t d = 0; d < detections.size(); ++d) {
        if (!detectionMatched[d]) {
            KalmanTrack track;
            track.id = nextTrackId_++;
            track.bbox = detections[d].bbox;
            track.measuredCenter = detections[d].center;
            track.age = 1;
            track.missedFrames = 0;

            track.kf.init(detections[d].center.x, detections[d].center.y);
            track.filteredCenter = track.kf.getPosition();
            track.velocity = track.kf.getVelocity();

            tracks_.push_back(track);
        }
    }

    // Remove stale tracks
    std::vector<KalmanTrack> aliveTracks;
    aliveTracks.reserve(tracks_.size());

    for (const auto& track : tracks_) {
        if (track.missedFrames <= maxMissedFrames_) {
            aliveTracks.push_back(track);
        }
    }

    tracks_ = aliveTracks;
    return tracks_;
}