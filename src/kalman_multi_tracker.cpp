#include "kalman_multi_tracker.hpp"
#include <cmath>
#include <limits>

// KalmanMultiTracker:
// Maintains multiple tracks, each with its own Kalman filter.
// Uses greedy nearest-neighbor association between predicted tracks
// and current detections.
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
    // First predict all existing tracks forward in time.
    for (auto& track : tracks_) {
        track.kf.predict(dt);
        track.filteredCenter = track.kf.getPosition();
        track.velocity = track.kf.getVelocity();
    }

    std::vector<bool> detectionMatched(detections.size(), false);
    std::vector<bool> trackMatched(tracks_.size(), false);

    // Greedy nearest-neighbor association:
    // match each predicted track to the closest unmatched detection.
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

            // Update the Kalman filter using the matched measurement.
            tracks_[t].kf.update(det.center);
            tracks_[t].filteredCenter = tracks_[t].kf.getPosition();
            tracks_[t].velocity = tracks_[t].kf.getVelocity();

            tracks_[t].age += 1;
            tracks_[t].missedFrames = 0;

            detectionMatched[bestDetectionIndex] = true;
            trackMatched[t] = true;
        }
    }

    // Tracks without a match are kept alive temporarily.
    for (size_t t = 0; t < tracks_.size(); ++t) {
        if (!trackMatched[t]) {
            tracks_[t].age += 1;
            tracks_[t].missedFrames += 1;
            tracks_[t].filteredCenter = tracks_[t].kf.getPosition();
            tracks_[t].velocity = tracks_[t].kf.getVelocity();
        }
    }

    // Create new tracks for unmatched detections.
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

    // Remove tracks that have been unmatched for too long.
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
void KalmanMultiTracker::updateWithRadar(const std::vector<RadarDetection>& radarDetections) {
    std::vector<bool> radarMatched(radarDetections.size(), false);

    // Associate each track with the nearest radar detection
    for (auto& track : tracks_) {
        float bestDistance = std::numeric_limits<float>::max();
        int bestRadarIndex = -1;

        for (size_t r = 0; r < radarDetections.size(); ++r) {
            if (radarMatched[r]) {
                continue;
            }

            float dist = distance(track.filteredCenter, radarDetections[r].position);
            if (dist < bestDistance && dist < maxMatchDistance_) {
                bestDistance = dist;
                bestRadarIndex = static_cast<int>(r);
            }
        }

        if (bestRadarIndex >= 0) {
            const auto& rd = radarDetections[bestRadarIndex];

            // EKF radar correction using nonlinear [range, angle]
            track.kf.updateRadar(rd.range, rd.angle);
            track.filteredCenter = track.kf.getPosition();
            track.velocity = track.kf.getVelocity();

            radarMatched[bestRadarIndex] = true;
        }
    }
}

std::vector<KalmanTrack> KalmanMultiTracker::getTracks() const {
    return tracks_;
}
