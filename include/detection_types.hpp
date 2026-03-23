#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

struct Detection {
    cv::Rect bbox;
    cv::Point2f center;
    double area = 0.0;
};