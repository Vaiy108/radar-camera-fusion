#include "motion_detector.hpp"

MotionDetector::MotionDetector()
    : bgSubtractor_(cv::createBackgroundSubtractorMOG2()),
      minArea_(800) {
}

std::vector<Detection> MotionDetector::detect(const cv::Mat& frame) {
    std::vector<Detection> detections;

    if (frame.empty()) {
        return detections;
    }

    cv::Mat fgMask;
    bgSubtractor_->apply(frame, fgMask);

    // Clean up noise
    cv::GaussianBlur(fgMask, fgMask, cv::Size(5, 5), 0);
    cv::threshold(fgMask, fgMask, 200, 255, cv::THRESH_BINARY);

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
        cv::Point2f center(
            bbox.x + bbox.width * 0.5f,
            bbox.y + bbox.height * 0.5f
        );

        detections.push_back({bbox, center, area});
    }

    return detections;
}