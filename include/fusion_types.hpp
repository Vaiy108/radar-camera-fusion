#pragma once

#include <opencv2/opencv.hpp>

struct FusedTrack {
    int id = -1;

    cv::Point2f cameraPosition;
    cv::Point2f radarPosition;
    cv::Point2f fusedPosition;

    bool hasCamera = false;
    bool hasRadar = false;
};