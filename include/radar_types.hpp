#pragma once

#include <opencv2/opencv.hpp>

struct RadarDetection {
    int associatedTrackId = -1;
    cv::Point2f position;
    float range = 0.0f;
    float angle = 0.0f;
};