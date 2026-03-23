#include "simple_tracker.hpp"
#include <cmath>
#include <limits>

SimpleTracker::SimpleTracker(float maxMatchDistance, int maxMissedFrames)
    : maxMatchDistance_(maxMatchDistance),
      maxMissedFrames_(maxMissedFrames),
      nextTrackId_(0) {
}

float SimpleTracker::distance(const cv::Point2f& a, const cv::Point2f& b) const {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<Track> SimpleTracker::update(const std::vector<Detection>& detections) {
    std::vector<bool> detectionMatched(detections.size(), false);
    std::vector<bool> trackMatched(tracks_.size(), false);

    // Greedy nearest-neighbor matching
    for (size_t t = 0; t < tracks_.size(); ++t) {
        float bestDistance = std::numeric_limits<float>::max();
        int bestDetectionIndex = -1;

        for (size_t d = 0; d < detections.size(); ++d) {
            if (detectionMatched[d]) {
                continue;
            }

            float dist = distance(tracks_[t].center, detections[d].center);
            if (dist < bestDistance && dist < maxMatchDistance_) {
                bestDistance = dist;
                bestDetectionIndex = static_cast<int>(d);
            }
        }

        if (bestDetectionIndex >= 0) {
            const auto& det = detections[bestDetectionIndex];
            tracks_[t].bbox = det.bbox;
            tracks_[t].center = det.center;
            tracks_[t].age += 1;
            tracks_[t].missedFrames = 0;

            detectionMatched[bestDetectionIndex] = true;
            trackMatched[t] = true;
        }
    }

    // Increment missedFrames for unmatched tracks
    for (size_t t = 0; t < tracks_.size(); ++t) {
        if (!trackMatched[t]) {
            tracks_[t].age += 1;
            tracks_[t].missedFrames += 1;
        }
    }

    // Create new tracks for unmatched detections
    for (size_t d = 0; d < detections.size(); ++d) {
        if (!detectionMatched[d]) {
            Track newTrack;
            newTrack.id = nextTrackId_++;
            newTrack.bbox = detections[d].bbox;
            newTrack.center = detections[d].center;
            newTrack.age = 1;
            newTrack.missedFrames = 0;
            tracks_.push_back(newTrack);
        }
    }

    // Remove dead tracks
    std::vector<Track> aliveTracks;
    aliveTracks.reserve(tracks_.size());

    for (const auto& track : tracks_) {
        if (track.missedFrames <= maxMissedFrames_) {
            aliveTracks.push_back(track);
        }
    }

    tracks_ = aliveTracks;
    return tracks_;
}