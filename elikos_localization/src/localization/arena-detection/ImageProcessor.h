//
// Created by olivier on 10/2/16.
//

#ifndef LOCALIZATION_IMAGE_PROCESSOR_H
#define LOCALIZATION_IMAGE_PROCESSOR_H

#include <opencv2/core/core.hpp>
#include "Line.h"
#include <Eigen/Core>
#include "PerspectiveTransform.h"

namespace localization 
{

class LineGroup;

class ImageProcessor
{
public:

    static ImageProcessor* getInstance();
    static void freeInstance();

    void processImage(cv::Mat input);

    void perspectiveToOrtho(double theta);

    double theta_ = 0.0;

    Eigen::Vector3f imuOrientation_ = { 1.0, 0.0, 0.0 };

private:

    static ImageProcessor* instance_;

    cv::Mat image_;

    cv::Mat distortionMap1_;
    cv::Mat distortionMap2_;

    std::vector<Line> lineCluster_;
    std::vector<Line> detectedLines_;

    cv::Mat lines_, lineGroups_, mLines_, intersectionGroup_;

    std::vector<Eigen::Vector2f> intersections_;

    cv::Point corners[4];

    PerspectiveTransform transform_;
    int C_W;
    int C_H;

    void preProcess(const cv::Mat& raw, cv::Mat& preProcessed);
    void findEdges(const cv::Mat& src, cv::Mat& edges);
    void findLines(const cv::Mat& edges, cv::Mat& line);

    void analyzeLineCluster();
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

    ImageProcessor();
    ~ImageProcessor() = default;
};

}

#endif // LOCALIZATION_IMAGE_PROCESSOR_H
