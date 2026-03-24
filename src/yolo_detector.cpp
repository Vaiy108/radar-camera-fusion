#include "yolo_detector.hpp"
#include <iostream>

// YoloDetector:
// Loads an ONNX YOLO model and performs object detection using OpenCV DNN.
// The output is converted into the project's generic Detection structure.
YoloDetector::YoloDetector(const std::string& modelPath,
                           float confThreshold,
                           float scoreThreshold,
                           float nmsThreshold,
                           int inputWidth,
                           int inputHeight)
    : confThreshold_(confThreshold),
      scoreThreshold_(scoreThreshold),
      nmsThreshold_(nmsThreshold),
      inputWidth_(inputWidth),
      inputHeight_(inputHeight),
      loaded_(false) {
    try {
        net_ = cv::dnn::readNetFromONNX(modelPath);
        loaded_ = !net_.empty();

        if (loaded_) {
            std::cout << "YOLO model loaded: " << modelPath << std::endl;
        } else {
            std::cerr << "Failed to load YOLO model: " << modelPath << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception while loading YOLO model: " << e.what() << std::endl;
        loaded_ = false;
    }
}

bool YoloDetector::isLoaded() const {
    return loaded_;
}

std::vector<Detection> YoloDetector::detect(const cv::Mat& frame) {
    std::vector<Detection> detections;

    if (!loaded_ || frame.empty()) {
        return detections;
    }

    // Preprocess the frame into a normalized blob for YOLO input.
    cv::Mat blob;
    cv::dnn::blobFromImage(frame, blob, 1.0 / 255.0,
                           cv::Size(inputWidth_, inputHeight_),
                           cv::Scalar(), true, false);

    net_.setInput(blob);

    std::vector<cv::Mat> outputs;
    net_.forward(outputs, net_.getUnconnectedOutLayersNames());

    if (outputs.empty()) {
        return detections;
    }

    cv::Mat output = outputs[0];

    // Depending on the ONNX export, the output may come as a 3D tensor.
    // Reshape to a 2D matrix where each row corresponds to one candidate box.
    if (output.dims == 3) {
        output = output.reshape(1, output.size[1]);
    }

    const int rows = output.rows;
    const int dimensions = output.cols;

    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    const float xFactor = static_cast<float>(frame.cols) / inputWidth_;
    const float yFactor = static_cast<float>(frame.rows) / inputHeight_;

    for (int i = 0; i < rows; ++i) {
        const float* data = output.ptr<float>(i);

        // Objectness confidence
        float objConf = data[4];
        if (objConf < confThreshold_) {
            continue;
        }

        // Class scores start after the first 5 values:
        // [cx, cy, w, h, objectness, class_scores...]
        cv::Mat scores(1, dimensions - 5, CV_32F, (void*)(data + 5));
        cv::Point classIdPoint;
        double maxClassScore;
        cv::minMaxLoc(scores, nullptr, &maxClassScore, nullptr, &classIdPoint);

        float finalScore = objConf * static_cast<float>(maxClassScore);
        if (finalScore < scoreThreshold_) {
            continue;
        }

        float cx = data[0];
        float cy = data[1];
        float w  = data[2];
        float h  = data[3];

        int left   = static_cast<int>((cx - 0.5f * w) * xFactor);
        int top    = static_cast<int>((cy - 0.5f * h) * yFactor);
        int width  = static_cast<int>(w * xFactor);
        int height = static_cast<int>(h * yFactor);

        boxes.emplace_back(left, top, width, height);
        confidences.push_back(finalScore);
        classIds.push_back(classIdPoint.x);
    }

    // Non-Maximum Suppression removes overlapping duplicate detections.
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, scoreThreshold_, nmsThreshold_, indices);

    for (int idx : indices) {
        cv::Rect bbox = boxes[idx] & cv::Rect(0, 0, frame.cols, frame.rows);
        if (bbox.width <= 0 || bbox.height <= 0) {
            continue;
        }

        cv::Point2f center(
            bbox.x + bbox.width * 0.5f,
            bbox.y + bbox.height * 0.5f
        );

        Detection det;
        det.bbox = bbox;
        det.center = center;
        det.area = static_cast<double>(bbox.area());

        detections.push_back(det);
    }

    return detections;
}