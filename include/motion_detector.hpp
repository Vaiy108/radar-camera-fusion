#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include "detection_types.hpp"

class MotionDetector {
public:
    MotionDetector();

    std::vector<Detection> detect(const cv::Mat& frame);

private:
    cv::Ptr<cv::BackgroundSubtractor> bgSubtractor_;
    int minArea_;
};