#pragma once

#include <opencv2/opencv.hpp>
#include "kalman_filter_2d.hpp"

struct KalmanTrack {
    int id = -1;
    cv::Rect bbox;
    cv::Point2f measuredCenter;
    cv::Point2f filteredCenter;
    cv::Point2f velocity;
    int age = 0;
    int missedFrames = 0;

    KalmanFilter2D kf;
};