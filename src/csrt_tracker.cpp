// #include "csrt_tracker.hpp" //Use only for CSRT mode, it needs opencv2/tracking.hpp
#include <iostream>

// Construct an uninitialized CSRT tracker wrapper.
CSRTTracker::CSRTTracker() : initialized_(false) {
}

// Initialize tracker by asking the user to select a region of interest
// in the first frame. This makes CSRT mode ideal for single-object demos
// such as tracking one race car in a video.
bool CSRTTracker::init(const cv::Mat& frame) {
    std::cout << "Select object to track and press ENTER." << std::endl;
    std::cout << "Press ESC or select an empty region to cancel." << std::endl;

    bbox_ = cv::selectROI("CSRT Tracker", frame, false, false);

    // Reject invalid or empty selection.
    if (bbox_.width <= 0 || bbox_.height <= 0) {
        std::cerr << "Invalid ROI selected." << std::endl;
        return false;
    }

    // Create the OpenCV CSRT tracker and initialize it with the selected box.
    tracker_ = cv::TrackerCSRT::create();
    tracker_->init(frame, bbox_);

    initialized_ = true;
    return true;
}

// Update the tracker state for the current frame.
// Returns true if tracking succeeds and the bounding box is updated.
bool CSRTTracker::update(const cv::Mat& frame) {
    if (!initialized_) {
        return false;
    }

    return tracker_->update(frame, bbox_);
}

// Draw the tracked bounding box and a label on the frame.
void CSRTTracker::draw(cv::Mat& frame) const {
    if (!initialized_) {
        return;
    }

    cv::rectangle(frame, bbox_, cv::Scalar(0, 255, 255), 2);

    cv::putText(frame,
                "CSRT Tracker",
                cv::Point(static_cast<int>(bbox_.x), static_cast<int>(bbox_.y) - 10),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                cv::Scalar(0, 255, 255),
                2);
}