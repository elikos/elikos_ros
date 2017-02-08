#ifndef SHAPEDETECTION_H
#define SHAPEDETECTION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "RobotDesc.h"

class ShapeDetection {
   public:
    ShapeDetection();
    void detect(const cv::Mat& input, cv::Mat& output_w, cv::Mat& output_r,
                cv::Mat& output_g, cv::Mat& output,
                std::vector<RobotDesc>& outputRobotsArray);
};
#endif