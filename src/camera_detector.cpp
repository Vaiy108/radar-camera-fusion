#include "camera_detector.hpp"

#include "motion_detector.hpp"
#include "yolo_detector.hpp"
#include "kalman_multi_tracker.hpp"
#include "radar_simulator.hpp"
//#include "csrt_tracker.hpp"

#include <iostream>
#include <string>

// Runtime mode switches.
// - CSRT mode is intended for single-object tracking videos such as the race car clip.
// - Fusion mode is intended for multi-object traffic scenes.
// - YOLO remains optional because it is heavier and may introduce latency on CPU.
namespace {
    const bool kUseCSRTMode = false;      // true  -> single-object CSRT mode
    const bool kUseYoloDetector = false; // true  -> YOLO in fusion mode, false -> motion detector

    // Temporary absolute path for local development.
    // Replace with a project-relative or configurable path before publishing.
    const std::string kYoloModelPath =
        "C:/Projects_sf/radar-camera-fusion/models/yolov8n.onnx";
}

// Open a live camera stream by index.
bool CameraDetector::open(int cameraIndex) {
    cap_.open(cameraIndex);
    if (!cap_.isOpened()) {
        std::cerr << "Failed to open camera index " << cameraIndex << std::endl;
        return false;
    }

    // Reduce camera resolution and buffering to improve responsiveness.
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

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

// Main demo loop.
// This file supports two modes:
//
// 1. CSRT mode:
//    - user selects one object in the first frame
//    - OpenCV CSRT tracks it across the sequence
//
// 2. Fusion mode:
//    - detect multiple objects using motion detection or YOLO
//    - track them with a Kalman-based multi-object tracker
//    - simulate radar measurements
//    - apply EKF-style radar correction
void CameraDetector::runDemo() {
    if (!cap_.isOpened()) {
        std::cerr << "Capture is not opened!" << std::endl;
        return;
    }

    MotionDetector motionDetector;
    YoloDetector yoloDetector(kYoloModelPath);
    KalmanMultiTracker tracker;
    RadarSimulator radarSimulator;
    // CSRTTracker csrtTracker; //Comment out when using CSRT mode

    // bool csrtInitialized = false; //Comment out when using CSRT mode

    // If YOLO is enabled, ensure the ONNX model is available before entering the loop.
    if (!kUseCSRTMode && kUseYoloDetector && !yoloDetector.isLoaded()) {
        std::cerr << "YOLO detector failed to load from: " << kYoloModelPath << std::endl;
        return;
    }

    // Estimate time step from the video frame rate.
    // This improves Kalman prediction compared with using a fixed 1.0f step.
    double fps = cap_.get(cv::CAP_PROP_FPS);
    if (fps <= 0.0) {
        fps = 30.0;
    }
    float dt = static_cast<float>(1.0 / fps);

    // Create a resizable display window for large videos.
    cv::namedWindow("CameraDetector", cv::WINDOW_NORMAL);
    cv::resizeWindow("CameraDetector", 900, 500);
    cv::moveWindow("CameraDetector", 50, 50);

    cv::Mat frame;

    while (true) {
        if (!cap_.read(frame) || frame.empty()) {
            std::cout << "End of stream or failed to read frame." << std::endl;
            break;
        }

        
        // // MODE 1: Single-object CSRT tracking
        
        // if (kUseCSRTMode) {
        //     // Initialize the tracker only once using manual ROI selection.
        //     if (!csrtInitialized) {
        //         if (!csrtTracker.init(frame)) {
        //             std::cerr << "CSRT initialization failed." << std::endl;
        //             return;
        //         }
        //         csrtInitialized = true;
        //     }

        //     // Update and draw the tracked bounding box.
        //     csrtTracker.update(frame);
        //     csrtTracker.draw(frame);

        //     cv::putText(frame,
        //                 "Mode: CSRT (Single Object)",
        //                 cv::Point(20, 30),
        //                 cv::FONT_HERSHEY_SIMPLEX, 0.8,
        //                 cv::Scalar(255, 255, 0), 2);

        //     cv::imshow("CameraDetector", frame);

        //     // Slightly slower playback makes prerecorded demos easier to watch.
        //     char key = static_cast<char>(cv::waitKey(30));
        //     if (key == 'q' || key == 27) {
        //         break;
        //     }

        //     // Skip the fusion pipeline when CSRT mode is active.
        //     continue;
        // }

       
        // MODE 2: Multi-object fusion pipeline
        

        // Choose the detector.
        // Both detectors return the same Detection format, so the rest of the
        // pipeline stays unchanged.
        std::vector<Detection> detections;
        if (kUseYoloDetector) {
            detections = yoloDetector.detect(frame);
        } else {
            detections = motionDetector.detect(frame);
        }

        // Step 1: Update the Kalman-based multi-object tracker using camera detections.
        auto tracks = tracker.update(detections, dt);

        // Convert filtered tracks into a simpler representation for radar simulation.
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

        // Step 2: Generate synthetic radar detections near the current tracked positions.
        auto radarDetections = radarSimulator.simulate(basicTracks);

        // Step 3: Use radar range/angle to perform an EKF-style correction.
        tracker.updateWithRadar(radarDetections);

        // Refresh the updated tracks after radar correction.
        tracks = tracker.getTracks();

        // Draw the final filtered tracks.
        for (const auto& track : tracks) {
            // Suppress very young tracks to reduce clutter from unstable IDs.
            if (track.age < 8) {
                continue;
            }

            // Bounding box from the camera detection.
            cv::rectangle(frame, track.bbox, cv::Scalar(0, 255, 0), 2);

            // Raw camera measurement.
            cv::circle(frame, track.measuredCenter, 4, cv::Scalar(0, 255, 0), -1);

            // EKF-filtered state estimate.
            cv::circle(frame, track.filteredCenter, 6, cv::Scalar(0, 255, 255), -1);

            // Estimated velocity vector.
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

        // Draw simulated radar detections.
        for (const auto& rd : radarDetections) {
            cv::circle(frame, rd.position, 5, cv::Scalar(255, 0, 0), 2);

            cv::putText(frame, "Radar",
                        cv::Point(static_cast<int>(rd.position.x) + 6,
                                  static_cast<int>(rd.position.y) - 6),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45,
                        cv::Scalar(255, 0, 0), 1);
        }

        // Show which detection pipeline is active.
        const std::string detectorText =
            kUseYoloDetector ? "Detector: YOLO" : "Detector: Motion";

        cv::putText(frame,
                    "Mode: Fusion (Multi Object)",
                    cv::Point(20, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(255, 255, 0), 2);

        cv::putText(frame,
                    detectorText,
                    cv::Point(20, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(255, 255, 0), 2);

        cv::putText(frame,
                    "Detections: " + std::to_string(detections.size()) +
                    " Tracks: " + std::to_string(tracks.size()) +
                    " Radar: " + std::to_string(radarDetections.size()),
                    cv::Point(20, 90),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(255, 255, 0), 2);

        cv::imshow("CameraDetector", frame);
        // Create a resizable display window for large videos.
        // cv::Mat displayFrame;
        // cv::resize(frame, displayFrame, cv::Size(), 0.9, 0.9);
        // cv::imshow("CameraDetector", displayFrame);

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