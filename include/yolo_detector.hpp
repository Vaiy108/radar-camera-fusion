#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <string>
#include <vector>
#include "detection_types.hpp"

// YoloDetector:
// Wraps an ONNX YOLO model loaded through OpenCV DNN and returns
// detections in the same Detection format used by the motion detector.

class YoloDetector {
public:
    YoloDetector(const std::string& modelPath,
                 float confThreshold = 0.4f,
                 float scoreThreshold = 0.25f,
                 float nmsThreshold = 0.45f,
                 int inputWidth = 320,
                 int inputHeight = 320);

    bool isLoaded() const;

    std::vector<Detection> detect(const cv::Mat& frame);

private:
    cv::dnn::Net net_;
    float confThreshold_;
    float scoreThreshold_;
    float nmsThreshold_;
    int inputWidth_;
    int inputHeight_;
    bool loaded_;
};