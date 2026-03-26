#pragma once

#include <opencv2/opencv.hpp>
//#include <opencv2/tracking.hpp> // use this for CSRT mode

// CSRTTracker:
// Simple wrapper around OpenCV's CSRT single-object tracker.
// This mode is useful for constrained scenes where one object is selected
// manually in the first frame and then tracked across the sequence.
class CSRTTracker {
public:
    CSRTTracker();

    // Initialize the tracker using a manually selected ROI
    // on the first frame.
    bool init(const cv::Mat& frame);

    // Update the tracker for the next frame.
    bool update(const cv::Mat& frame);

    // Draw the tracked bounding box and label.
    void draw(cv::Mat& frame) const;

private:
    cv::Ptr<cv::Tracker> tracker_;
    cv::Rect2d bbox_;
    bool initialized_;
};