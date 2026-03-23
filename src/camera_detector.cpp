#include "camera_detector.hpp"
#include "motion_detector.hpp"
#include "simple_tracker.hpp"
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
    SimpleTracker tracker;

    cv::Mat frame;

    while (true) {
        if (!cap_.read(frame) || frame.empty()) {
            std::cout << "End of stream or failed to read frame." << std::endl;
            break;
        }

        auto detections = detector.detect(frame);
        auto tracks = tracker.update(detections);

        for (const auto& track : tracks) {
            cv::rectangle(frame, track.bbox, cv::Scalar(0, 255, 0), 2);
            cv::circle(frame, track.center, 4, cv::Scalar(0, 0, 255), -1);

            std::string label = "ID " + std::to_string(track.id);
            cv::putText(frame, label,
                        cv::Point(track.bbox.x, track.bbox.y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0, 255, 0), 2);

            std::string meta = "age=" + std::to_string(track.age) +
                               " miss=" + std::to_string(track.missedFrames);

            cv::putText(frame, meta,
                        cv::Point(track.bbox.x, track.bbox.y + track.bbox.height + 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45,
                        cv::Scalar(255, 255, 0), 1);
        }

        cv::putText(frame,
                    "Tracks: " + std::to_string(tracks.size()),
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