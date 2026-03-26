#include "camera_detector.hpp"
//suppress OpenCV info logging
#include <opencv2/core/utils/logger.hpp>


int main() {
    // Reduce OpenCV console noise from optional backend/plugin probing.
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);

    CameraDetector detector;

    // Choose input source here:
    // Option 1: webcam
    // if (!detector.open(0)) {
    //     return -1;
    // }

    // Option 2: traffic video for multi-object fusion
    if (!detector.open("C:/Projects_sf/radar-camera-fusion/data/traffic.mp4")) {
        return -1;
    }

    // Option 3: race car video for CSRT single-object tracking
    // if (!detector.open("C:/Projects_sf/radar-camera-fusion/data/racecar.mp4")) {
    //     return -1;
    // }

    detector.runDemo();

    return 0;
}






// #include <opencv2/opencv.hpp>
// #include <iostream>

// int main() {
//     cv::Mat img = cv::Mat::zeros(400, 400, CV_8UC3);

//     cv::putText(img, "OpenCV Linked!", cv::Point(50, 200),
//                 cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

//     cv::imshow("Test", img);
//     cv::waitKey(0);
//     return 0;
// }