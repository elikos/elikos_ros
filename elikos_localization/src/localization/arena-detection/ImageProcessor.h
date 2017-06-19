//
// Created by olivier on 10/2/16.
//

#ifndef LOCALIZATION_IMAGE_PROCESSOR_H
#define LOCALIZATION_IMAGE_PROCESSOR_H

#include <fstream>
#include <ros/time.h>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>

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

    ImageProcessor(QuadState* state);
    ~ImageProcessor();

    void processImage(const cv::Mat& input, const ros::Time& stamp);

    void perspectiveToOrtho(double theta);

    void updateQuadImu();

    double roll_ = 0.0;
    double pitch_ = 0.0;

    Eigen::Vector3d imuOrientation_ = { 1.0, 0.0, 0.0 };

private:
    cv::Mat image_;

    cv::Mat distortionMap1_;
    cv::Mat distortionMap2_;

    std::vector<Line> lineCluster_;
    std::vector<Line> detectedLines_;

    cv::Mat lines_, mLines_, preProcessed_;

    std::vector<Eigen::Vector2f> intersections_;

    cv::Point corners[4];

    PerspectiveTransform transform_;
    PreProcessing preProcessing_;
    LineDetection lineDetection_;
    GridFitting gridFitting_;
    IntersectionTransform intersectionTransform_;

    int C_W;
    int C_H;

    ros::Time start_;
    std::ofstream file_;

    void preProcess(const cv::Mat& raw, cv::Mat& preProcessed);
    void findEdges(const cv::Mat& src, cv::Mat& edges);
    void findLines(const cv::Mat& edges, cv::Mat& line);

    void analyzeLineCluster(ros::Time stamp);
    void buildLineArray(const std::vector<cv::Vec2f>& lineCluster);

    void groupByOrientation(std::vector<LineGroup>& orientationGroup, const std::vector<Line>& lines);
    void groupByOrientation(std::vector<LineGroup>& group, Line& line);

    void parseClusterMemberships(const std::vector<int>& clusterMemberships, std::vector<Eigen::Vector2f>& intersections);
    void findLineIntersections(const std::vector<LineGroup>& orientationGroups);
    void findLineIntersections(const LineGroup& firstGroup, const LineGroup otherGroup);

    void findLineIntersections();

    void drawIntersection(const std::vector<Eigen::Vector2f>& intersections, const cv::Scalar& color);
    void drawRawLines(cv::Mat& dst, const std::vector<cv::Vec2f> &raw_lines) const;
    void drawLines(cv::Mat& dst, const std::vector<Line>& lines) const;
    void drawLine(cv::Mat& dst, const Line& line, const cv::Scalar& color) const;
    void drawLineGroup(cv::Mat& dst, const LineGroup& group, const cv::Scalar& color);

};

}

#endif // LOCALIZATION_IMAGE_PROCESSOR_H
