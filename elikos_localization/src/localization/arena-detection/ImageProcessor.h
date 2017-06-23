//
// Created by olivier on 10/2/16.
//

#ifndef LOCALIZATION_IMAGE_PROCESSOR_H
#define LOCALIZATION_IMAGE_PROCESSOR_H

#include <fstream>
#include <ros/time.h>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>

#include "CameraInfo.h"
#include "PreProcessing.h"
#include "PerspectiveTransform.h"
#include "GridFitting.h"
#include "LineDetection.h"
#include "IntersectionTransform.h"

#include "Line.h"
#include "AbstractKalmanFilter.h"

namespace localization 
{

class LineGroup;

class ImageProcessor
{
public:

    ImageProcessor(const CameraInfo& cameraInfo, const QuadState& state);
    ~ImageProcessor();

    void processImage(cv::Mat& input, cv::Mat& result);
private:

    std::vector<Line> lineCluster_;

    std::vector<Eigen::Vector2f> intersections_;

    PreProcessing preProcessing_;
    LineDetection lineDetection_;
    IntersectionTransform intersectionTransform_;

    int C_W;
    int C_H;

    void findEdges(const cv::Mat& src, cv::Mat& edges);
    void findLines(const cv::Mat& edges);

    void analyzeLineCluster(cv::Mat& debug);
    void buildLineArray(const std::vector<cv::Vec2f>& lineCluster);

    void groupByOrientation(std::vector<LineGroup>& orientationGroup, const std::vector<Line>& lines);
    void groupByOrientation(std::vector<LineGroup>& group, Line& line);

    void parseClusterMemberships(const std::vector<int>& clusterMemberships, std::vector<Eigen::Vector2f>& intersections);
    void findLineIntersections(const std::vector<LineGroup>& orientationGroups);
    void findLineIntersections(const LineGroup& firstGroup, const LineGroup otherGroup);

    void drawIntersection(const std::vector<Eigen::Vector2f>& intersections, const cv::Scalar& color, cv::Mat& dst);
    void drawLine(cv::Mat& dst, const Line& line, const cv::Scalar& color) const;
};

}

#endif // LOCALIZATION_IMAGE_PROCESSOR_H
