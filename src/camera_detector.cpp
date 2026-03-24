//#include "fusion_manager.hpp"

#include "camera_detector.hpp"
#include "motion_detector.hpp"
#include "yolo_detector.hpp"
#include "kalman_multi_tracker.hpp"
#include "radar_simulator.hpp"

#include <iostream>
#include <string>

// Toggle between classical motion-based detection and YOLO-based detection.
// true  -> use YOLO detector
// false -> use MotionDetector
namespace {
    //constexpr bool kUseYoloDetector = true; // YOLO detector mode

    constexpr bool kUseYoloDetector = false; //- Uncomment to switch to MotionDetector

    // Path to the ONNX model used by OpenCV DNN.
    const std::string kYoloModelPath = "C:/Projects_sf/radar-camera-fusion/models/yolov8n.onnx";
}

// Open a live camera stream by index.
bool CameraDetector::open(int cameraIndex) {
    cap_.open(cameraIndex);
    if (!cap_.isOpened()) {
        std::cerr << "Failed to open camera index " << cameraIndex << std::endl;
        return false;
    }
    return true;
}

// Open a prerecorded video file.
bool CameraDetector::open(const std::string& videoPath) {
    cap_.open(videoPath);
    if (!cap_.isOpened()) {
        std::cerr << "Failed to open video: " << videoPath << std::endl;
        return false;
    }
    return true;
}

// Main demo loop:
// CONCEPT:
// filtered point = state estimate corrected by camera + radar
// radar update is nonlinear - much closer to real radar-camera fusion logic
// 1. capture frame
// 2. detect objects using either motion detection or YOLO
// 3. track objects with a Kalman-based multi-object tracker
// 4. simulate radar measurements
// 5. apply EKF-style radar correction
// 6. visualize camera measurements, filtered tracks, radar points, and velocity
void CameraDetector::runDemo() {
    if (!cap_.isOpened()) {
        std::cerr << "Capture is not opened!" << std::endl;
        return;
    }

    MotionDetector motionDetector;
    YoloDetector yoloDetector(kYoloModelPath);
    KalmanMultiTracker tracker;
    RadarSimulator radarSimulator;

    // If YOLO mode is enabled, verify that the ONNX model was loaded successfully.
    if (kUseYoloDetector && !yoloDetector.isLoaded()) {
        std::cerr << "YOLO detector failed to load from: " << kYoloModelPath << std::endl;
        return;
    }

    cv::Mat frame;

    while (true) {
        if (!cap_.read(frame) || frame.empty()) {
            std::cout << "End of stream or failed to read frame." << std::endl;
            break;
        }

        // Choose the detection source using a single switch.
        // Both detectors return std::vector<Detection>, so the rest of the
        // pipeline can remain unchanged.
        std::vector<Detection> detections;
        if (kUseYoloDetector) {
            detections = yoloDetector.detect(frame);
        } else {
            detections = motionDetector.detect(frame);
        }

        // Step 1: Camera-based tracking update
        auto tracks = tracker.update(detections, 1.0f);

        // Convert KalmanTrack objects into simpler Track objects for radar simulation.
        // The radar simulator currently uses filtered track centers as pseudo ground truth.
        std::vector<Track> basicTracks;
        basicTracks.reserve(tracks.size());

        for (const auto& kt : tracks) {
            Track t;
            t.id = kt.id;
            t.bbox = kt.bbox;
            t.center = kt.filteredCenter;
            t.age = kt.age;
            t.missedFrames = kt.missedFrames;
            basicTracks.push_back(t);
        }

        // Step 2: Simulate radar detections
        auto radarDetections = radarSimulator.simulate(basicTracks);

        // Step 3: Apply EKF-style radar correction to the tracker
        tracker.updateWithRadar(radarDetections);

        // Refresh the latest corrected tracks after radar update
        tracks = tracker.getTracks();

        // Draw filtered tracks
        for (const auto& track : tracks) {
            // Suppress very young tracks to reduce clutter from unstable initial IDs.
            if (track.age < 5) {
                continue;
            }

            // Bounding box from the camera detection
            cv::rectangle(frame, track.bbox, cv::Scalar(0, 255, 0), 2);

            // Raw camera measurement
            cv::circle(frame, track.measuredCenter, 4, cv::Scalar(0, 255, 0), -1);

            // EKF-filtered state estimate
            cv::circle(frame, track.filteredCenter, 6, cv::Scalar(0, 255, 255), -1);

            // Estimated velocity vector
            cv::Point2f endPoint = track.filteredCenter + track.velocity * 2.0f;
            cv::line(frame, track.filteredCenter, endPoint, cv::Scalar(255, 0, 255), 2);

            std::string label = "EKF ID " + std::to_string(track.id);
            cv::putText(frame, label,
                        cv::Point(track.bbox.x, track.bbox.y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0, 255, 0), 2);

            std::string velocityLabel =
                "vx=" + std::to_string(static_cast<int>(track.velocity.x)) +
                " vy=" + std::to_string(static_cast<int>(track.velocity.y));
            cv::putText(frame, velocityLabel,
                        cv::Point(track.bbox.x, track.bbox.y + track.bbox.height + 18),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45,
                        cv::Scalar(255, 255, 0), 1);
        }

        // Draw radar detections
        for (const auto& rd : radarDetections) {
            cv::circle(frame, rd.position, 6, cv::Scalar(255, 0, 0), 2);

            std::string label = "Radar";
            cv::putText(frame, label,
                        cv::Point(static_cast<int>(rd.position.x) + 6,
                                  static_cast<int>(rd.position.y) - 6),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45,
                        cv::Scalar(255, 0, 0), 1);
        }

        // Draw detection mode in the top-left corner
        const std::string modeText = kUseYoloDetector ? "Detector: YOLO" : "Detector: Motion";
        cv::putText(frame,
                    modeText,
                    cv::Point(20, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(255, 255, 0), 2);

        // Draw counts below the mode text
        cv::putText(frame,
                    "Detections: " + std::to_string(detections.size()) +
                    " Tracks: " + std::to_string(tracks.size()) +
                    " Radar: " + std::to_string(radarDetections.size()),
                    cv::Point(20, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(255, 255, 0), 2);

        cv::imshow("CameraDetector", frame);

        char key = static_cast<char>(cv::waitKey(1));
        if (key == 'q' || key == 27) {
            break;
        }
    }
}




// Code for - without EKF
//CONCEPT: fused point = weighted average of camera and radar positions

// Main demo loop:
// 1. capture frame
// 2. detect motion
// 3. track objects with Kalman filtering
// 4. simulate radar detections
// 5. fuse camera and radar positions
// 6. visualize all intermediate outputs
// void CameraDetector::runDemo() {
//     if (!cap_.isOpened()) {
//         std::cerr << "Capture is not opened!" << std::endl;
//         return;
//     }

//     MotionDetector detector;
//     KalmanMultiTracker tracker;
//     RadarSimulator radarSimulator;
//     FusionManager fusionManager;

//     cv::Mat frame;

//     while (true) {
//         if (!cap_.read(frame) || frame.empty()) {
//             std::cout << "End of stream or failed to read frame." << std::endl;
//             break;
//         }

//         auto detections = detector.detect(frame);
//         auto tracks = tracker.update(detections, 1.0f);

//         // Convert KalmanTrack objects to simpler Track objects
//         // for the current radar simulation and fusion modules.
//         std::vector<Track> basicTracks;
//         basicTracks.reserve(tracks.size());

//         for (const auto& kt : tracks) {
//             Track t;
//             t.id = kt.id;
//             t.bbox = kt.bbox;
//             t.center = kt.filteredCenter;
//             t.age = kt.age;
//             t.missedFrames = kt.missedFrames;
//             basicTracks.push_back(t);
//         }

//         auto radarDetections = radarSimulator.simulate(basicTracks);
//         auto fusedTracks = fusionManager.fuse(basicTracks, radarDetections);

//         // Draw Kalman-based camera tracks.
//         for (const auto& track : tracks) {
//             // Suppress very young tracks to avoid visual clutter from unstable IDs.
//             if (track.age < 5) {
//                 continue;
//             }

//             cv::rectangle(frame, track.bbox, cv::Scalar(0, 255, 0), 2);

//             // Raw camera measurement
//             cv::circle(frame, track.measuredCenter, 4, cv::Scalar(0, 255, 0), -1);

//             // Kalman filtered state estimate
//             cv::circle(frame, track.filteredCenter, 6, cv::Scalar(0, 255, 255), -1);

//             // Estimated velocity vector
//             cv::Point2f endPoint = track.filteredCenter + track.velocity * 2.0f;
//             cv::line(frame, track.filteredCenter, endPoint, cv::Scalar(255, 0, 255), 2);

//             std::string label = "KF ID " + std::to_string(track.id);
//             cv::putText(frame, label,
//                         cv::Point(track.bbox.x, track.bbox.y - 10),
//                         cv::FONT_HERSHEY_SIMPLEX, 0.6,
//                         cv::Scalar(0, 255, 0), 2);
//         }

//         // Draw radar detections.
//         for (const auto& rd : radarDetections) {
//             cv::circle(frame, rd.position, 6, cv::Scalar(255, 0, 0), 2);
//             cv::putText(frame, "Radar",
//                         cv::Point(static_cast<int>(rd.position.x) + 6,
//                                   static_cast<int>(rd.position.y) - 6),
//                         cv::FONT_HERSHEY_SIMPLEX, 0.45,
//                         cv::Scalar(255, 0, 0), 1);
//         }

//         // Draw fused positions.
//         for (const auto& fused : fusedTracks) {
//             cv::circle(frame, fused.fusedPosition, 5, cv::Scalar(255, 255, 255), -1);

//             std::string label = "Fused " + std::to_string(fused.id);
//             cv::putText(frame, label,
//                         cv::Point(static_cast<int>(fused.fusedPosition.x) + 6,
//                                   static_cast<int>(fused.fusedPosition.y) + 14),
//                         cv::FONT_HERSHEY_SIMPLEX, 0.45,
//                         cv::Scalar(255, 255, 255), 1);
//         }

//         cv::putText(frame,
//                     "KF Tracks: " + std::to_string(tracks.size()) +
//                     " Radar: " + std::to_string(radarDetections.size()) +
//                     " Fused: " + std::to_string(fusedTracks.size()),
//                     cv::Point(20, 30),
//                     cv::FONT_HERSHEY_SIMPLEX, 0.8,
//                     cv::Scalar(255, 255, 0), 2);

//         cv::imshow("CameraDetector", frame);

//         char key = static_cast<char>(cv::waitKey(1));
//         if (key == 'q' || key == 27) {
//             break;
//         }
//     }
// }