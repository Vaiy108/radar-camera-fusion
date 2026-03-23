#include "camera_detector.hpp"
#include "motion_detector.hpp"
#include <iostream>
#include <string>

bool CameraDetector::open(int cameraIndex) {
    cap_.open(cameraIndex);
    if (!cap_.isOpened()) {
        std::cerr << "Failed to open camera index " << cameraIndex << std::endl;
        return false;
    }
    return true;
}

bool CameraDetector::open(const std::string& videoPath) {
    cap_.open(videoPath);
    if (!cap_.isOpened()) {
        std::cerr << "Failed to open video: " << videoPath << std::endl;
        return false;
    }
    return true;
}

void CameraDetector::runDemo() {
    if (!cap_.isOpened()) {
        std::cerr << "Capture is not opened!" << std::endl;
        return;
    }

    MotionDetector detector;
    cv::Mat frame;

    while (true) {
        if (!cap_.read(frame) || frame.empty()) {
            std::cout << "End of stream or failed to read frame." << std::endl;
            break;
        }

        auto detections = detector.detect(frame);

        for (size_t i = 0; i < detections.size(); ++i) {
            const auto& det = detections[i];

            cv::rectangle(frame, det.bbox, cv::Scalar(0, 255, 0), 2);
            cv::circle(frame, det.center, 4, cv::Scalar(0, 0, 255), -1);

            std::string label = "Obj " + std::to_string(i);
            cv::putText(frame, label,
                        cv::Point(det.bbox.x, det.bbox.y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0, 255, 0), 2);
        }

        cv::putText(frame,
                    "Detections: " + std::to_string(detections.size()),
                    cv::Point(20, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(255, 255, 0), 2);

        cv::imshow("CameraDetector", frame);

        char key = static_cast<char>(cv::waitKey(1));
        if (key == 'q' || key == 27) {
            break;
        }
    }
}