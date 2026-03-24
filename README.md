# Radar-Camera Fusion Tracker

A real-time C++ prototype for camera-based motion detection, multi-object tracking, Kalman filtering, simulated radar measurements, and basic camera-radar fusion visualization.

## Overview

This project was built as a portfolio prototype for algorithm engineering roles in computer vision, tracking, and sensor fusion.

The system processes live video input, detects moving objects, tracks them across frames, estimates object states with a constant-velocity Kalman filter, simulates radar detections, and fuses radar and camera information into a combined visualization.

The project is designed to demonstrate practical skills in:

- C++
- OpenCV
- multi-object tracking
- Bayesian filtering / Kalman filtering
- sensor fusion
- modular project structure with CMake

## Features

- Real-time webcam or video input
- Motion-based object detection using background subtraction
- Detection filtering to reduce noise and false positives
- Multi-object tracking with persistent track IDs
- Constant-velocity Kalman filter for state estimation
- Simulated radar detections with noise
- Camera-radar association and fused position visualization
- Modular C++ codebase organized with headers and source files
- Built using CMake and Visual Studio 2022 on Windows

## Current Pipeline

The current processing pipeline is:

1. Capture frame from camera
2. Detect moving objects using background subtraction
3. Extract object bounding boxes and centers
4. Track detections across frames
5. Predict and update object state using a Kalman filter
6. Simulate radar detections from tracked object positions
7. Associate radar detections with camera tracks
8. Compute fused object positions
9. Visualize camera, radar, and fused outputs

## Project Structure

```text
radar-camera-fusion/
├── CMakeLists.txt
├── CMakeSettings.json
├── README.md
├── include/
│   ├── camera_detector.hpp
│   ├── detection_types.hpp
│   ├── fusion_manager.hpp
│   ├── fusion_types.hpp
│   ├── kalman_filter_2d.hpp
│   ├── kalman_multi_tracker.hpp
│   ├── kalman_track.hpp
│   ├── motion_detector.hpp
│   ├── radar_simulator.hpp
│   ├── radar_types.hpp
│   ├── simple_tracker.hpp
│   └── track_types.hpp
└── src/
    ├── camera_detector.cpp
    ├── fusion_manager.cpp
    ├── kalman_filter_2d.cpp
    ├── kalman_multi_tracker.cpp
    ├── main.cpp
    ├── motion_detector.cpp
    ├── radar_simulator.cpp
    └── simple_tracker.cpp
```

## Technologies Used
- C++17
- OpenCV 4.x
- CMake
- Visual Studio 2022
- Ninja build system

## Build Instructions
### Prerequisites
- Windows 10 or later
- Visual Studio 2022 with Desktop development with C++
- OpenCV extracted locally, for example:
```
C:\opencv
```
- OpenCV runtime added to PATH:
```
C:\opencv\build\x64\vc16\bin
```
## Configure OpenCV

In CMakeLists.txt, OpenCV is referenced with:
```
set(OpenCV_DIR "C:/opencv/build/x64/vc16/lib")
find_package(OpenCV REQUIRED CONFIG)
```
## Build
Open the project folder in Visual Studio 2026
Run:
```
Project → Configure Cache
Build → Build All
Run the executable with:
Ctrl + F5
```
## Usage

By default, the system opens the default camera:
```
detector.open(0);
```
To use a video file instead, replace it with:
```
detector.open("path/to/video.mp4");
```
## Visualization Legend
- Green box: tracked object bounding box
- Green point: measured camera position
- Yellow point: Kalman filtered position
- Magenta line: estimated velocity direction
- Blue point: simulated radar detection
- White point: fused camera-radar estimate


## Example Output

The system displays:

- number of active Kalman tracks
- number of radar detections
- number of fused targets
- track IDs
- velocity estimates

## What I Learned

This project helped me practice and understand:

- how to structure a medium-size C++ vision project
- how detection noise affects tracking quality
- how to stabilize tracks using filtering and lifecycle logic
- how sensor measurements can be associated and fused
- how to debug OpenCV, CMake, and runtime issues in Visual Studio

## Current Limitations

This is a prototype and has several limitations:
- motion detection is sensitive to background noise and lighting changes
- radar is simulated rather than read from a real sensor
- radar fusion currently uses simplified positional association
- data association is greedy nearest-neighbor, not Hungarian or JPDA
- radar update is not yet a full nonlinear EKF with range-angle measurement model

## Future Improvements

Planned upgrades include:

- replace motion detection with YOLO or another object detector
- implement Hungarian assignment for more robust track matching
- extend radar model to range, angle, and radial velocity
- implement EKF-based radar update
- improve multi-target robustness when objects cross paths
- export results and performance metrics
- optimize for higher frame rates

## Why This Project Matters

This project is intended as a practical demonstration of skills relevant to:
- algorithm engineering
- computer vision
- real-time tracking
- radar-camera fusion
- embedded vision / edge AI pipelines