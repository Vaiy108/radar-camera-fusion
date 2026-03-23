#pragma once

#include <opencv2/opencv.hpp>
#include <string>

class CameraDetector {
public:
    // Open default camera by index, or a video file path
    bool open(int cameraIndex = 0);
    bool open(const std::string& videoPath);

    // Grab and show frames (for now, just display)
    void runDemo();

private:
    cv::VideoCapture cap_;
};