#pragma once

#include <opencv2/opencv.hpp>

struct Track {
    int id = -1;
    cv::Rect bbox;
    cv::Point2f center;
    int age = 0;               // total frames since creation
    int missedFrames = 0;      // consecutive unmatched frames
};