#include "motion_detector.hpp"
#include <algorithm>

// MotionDetector:
// Uses background subtraction to identify moving foreground regions,
// then extracts bounding boxes and centers for candidate objects.
MotionDetector::MotionDetector()
    : bgSubtractor_(cv::createBackgroundSubtractorMOG2()),
      minArea_(4000) {
}

// Detect moving objects in the current frame.
// Returns a filtered list of detections based on contour area and bbox size.
std::vector<Detection> MotionDetector::detect(const cv::Mat& frame) {
    std::vector<Detection> detections;

    if (frame.empty()) {
        return detections;
    }

    cv::Mat fgMask;

    // Apply background subtraction to separate moving foreground
    // from static background regions.
    bgSubtractor_->apply(frame, fgMask, 0.01);

    // Reduce mask noise before contour extraction.
    cv::GaussianBlur(fgMask, fgMask, cv::Size(5, 5), 0);
    cv::threshold(fgMask, fgMask, 200, 255, cv::THRESH_BINARY);

    // Morphological cleanup:
    // - OPEN removes small isolated blobs
    // - DILATE reconnects fragmented foreground regions
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(fgMask, fgMask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(fgMask, fgMask, cv::MORPH_DILATE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(fgMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < minArea_) {
            continue;
        }

        cv::Rect bbox = cv::boundingRect(contour);

        // Filter very small bounding boxes that are usually noise.
        if (bbox.width < 50 || bbox.height < 40) {
            continue;
        }

        cv::Point2f center(
            bbox.x + bbox.width * 0.5f,
            bbox.y + bbox.height * 0.5f
        );

        detections.push_back({bbox, center, area});
    }

    // Keep only the largest detections to reduce clutter and stabilize tracking.
    std::sort(detections.begin(), detections.end(),
              [](const Detection& a, const Detection& b) {
                  return a.area > b.area;
              });

    if (detections.size() > 5) {
        detections.resize(5);
    }

    return detections;
}