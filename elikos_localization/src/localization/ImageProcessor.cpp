//
// Created by olivier on 10/2/16.
//

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

#include "LineGroup.h"

#include "ImageProcessor.h"

namespace localization {

ImageProcessor* ImageProcessor::instance_ = nullptr;

ImageProcessor* ImageProcessor::getInstance()
{
    if (instance_ == nullptr)
    {
        instance_ = new ImageProcessor();
    }
    return instance_;
}

void ImageProcessor::freeInstance()
{
    delete instance_;
    instance_ = nullptr;
}

ImageProcessor::ImageProcessor()
{
    // Init undistortion map
    cv::Mat distortedCamera = (cv::Mat_<float>(3,3) << 422.918640,    0.000000,    350.119451,
            0.000000,  423.121112,    236.380265,
            0.000000,    0.000000,      1.000000);

    cv::Mat cameraDistortion = (cv::Mat_<float>(1,5) << -0.321590, 0.089597, 0.001090, -0.000489, 0.000000);

    cv::Mat undistortedCamera = cv::getOptimalNewCameraMatrix(distortedCamera, cameraDistortion, cv::Size(640, 480), 0);

    cv::initUndistortRectifyMap(distortedCamera, cameraDistortion, cv::Mat(), undistortedCamera,
                                cv::Size(640, 480), CV_32FC1, distortionMap1_, distortionMap2_);
}

void undistort(const cv::Mat& src, cv::Mat& undistorted)
{
}

void ImageProcessor::preProcess(const cv::Mat& raw, cv::Mat& preProcessed)
{
    //undistort
    // Blur
    cv::Mat typeConverted;
    if (raw.type() != CV_8UC1) {
        cv::cvtColor(raw, typeConverted, CV_BGR2GRAY);
    } else {
        raw.copyTo(typeConverted);
    }

    cv::Mat undistorted;
    cv::remap(typeConverted, undistorted, distortionMap1_, distortionMap2_, CV_INTER_LINEAR);

    cv::Mat blured;
    cv::GaussianBlur(undistorted, blured, cv::Size(7,7), 8, 8);

    cv::Mat eroded;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    erode(blured, eroded, element, cv::Point(0), 8);

    cv::Mat thresholded;
    cv::adaptiveThreshold(blured, thresholded, 200, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 551, 2);

    preProcessed = thresholded;
}

void ImageProcessor::processImage(cv::Mat input)
{
    image_ = input;
    cv::Mat preProcessed;
    preProcess(input, preProcessed);

    cv::Mat edges;
    findEdges(preProcessed, edges);

    cv::Mat lines = cv::Mat(input.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    vLines_ = cv::Mat(input.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    hLines_ = cv::Mat(input.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    lineGroups_ = cv::Mat(input.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    mLines_ = cv::Mat(input.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    intersectionGroup_ = cv::Mat(input.size(), CV_8UC3, cv::Scalar(0, 0, 0));

    findLines(edges, lines);
    drawLines(mLines_, lineCluster_);

    cv::imshow("input", preProcessed);
    cv::imshow("intersection", intersectionGroup_);
    cv::imshow("mLines", mLines_);
    cv::imshow("groups", lineGroups_);
    cv::imshow("lines", lines);
    cv::waitKey(0);
}
void ImageProcessor::findEdges(const cv::Mat& src, cv::Mat& edges)
{
    cv::Canny(src, edges, 1, 30, 3);
}


void ImageProcessor::drawLines(cv::Mat& dst, const std::vector<Line>& lines) const
{
    for (int i = 0; i < lines.size(); ++i)
    {
        drawLine(dst, lines[i], cv::Scalar(100, 100, 100));
    }
}

void ImageProcessor::drawLine(cv::Mat& dst, const Line& line, const cv::Scalar& color) const
{
    cv::Point2f centroid = line.getCentroid();
    cv::Vec2f orientation = line.getOrientation();

    cv::Point2f pt1, pt2;
    pt1.x = cvRound(centroid.x + 1000*(-orientation[0]));
    pt1.y = cvRound(centroid.y + 1000*(orientation[1]));
    pt2.x = cvRound(centroid.x - 1000*(-orientation[0]));
    pt2.y = cvRound(centroid.y - 1000*(orientation[1]));
    cv::line(dst, pt1, pt2, color, 3, CV_AA);
}

void ImageProcessor::drawRawLines(cv::Mat &dst, const std::vector<cv::Vec2f> &raw_lines) const {
    for( size_t i = 0; i < raw_lines.size() && i < 100; ++i )
    {
        float rho = raw_lines[i][0];
        float theta = raw_lines[i][1];

        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line(dst, pt1, pt2, cv::Scalar(100,100,100), 3, CV_AA);
    }
}

void ImageProcessor::drawLineGroup(cv::Mat& dst, const LineGroup& group, const cv::Scalar& color)
{
    std::vector<Line*> lines = group.getLines();
    for( size_t i = 0; i < lines.size() && i < 100; ++i )
    {
        drawLine(dst, *lines[i], color);
    }
}

void ImageProcessor::findLines(const cv::Mat& edges, cv::Mat& lines)
{
    std::vector<cv::Vec2f> rawLines;
    cv::HoughLines(edges, rawLines, 1, CV_PI/180, 100, 0, 0 );
    drawRawLines(lines, rawLines);

    buildLineArray(rawLines);
    analyzeLineCluster();

}

void ImageProcessor::analyzeLineCluster()
{
    if (lineCluster_.size() == 0) {
        return;
    }

    std::vector<LineGroup> orientationGroup;
    groupByOrientation(orientationGroup, lineCluster_);

    std::vector<LineGroup> intersectionGroup;
    groupByIntersection(intersectionGroup, orientationGroup);

    std::vector<LineGroup> distanceGroup;
    groupByDistance(distanceGroup, intersectionGroup);

    for (int i = 0; i < orientationGroup.size(); ++i)
    {
        int r = (i % 3 == 0) ? 100 : 0;
        int g = (i % 3 == 1) ? 100 : 0;
        int b = (i % 3 == 2) ? 100 : 0;

        drawLineGroup(lineGroups_, orientationGroup[i], cv::Scalar(r, g, b));
    }

    for(int i = 0; i < intersectionGroup.size(); ++i) {
        drawLineGroup(intersectionGroup_, intersectionGroup[i], cv::Scalar(100, 100, 100));
    }

    std::vector<LineGroup> totalDistanceGroup;
    for (int i = 0; i < distanceGroup.size(); ++i)
    {
        std::vector<LineGroup> distanceGroup;
        totalDistanceGroup.insert(totalDistanceGroup.end(), distanceGroup.begin(), distanceGroup.end());
    }

    detectedLines_.clear();
    for (int i = 0; i < totalDistanceGroup.size(); ++i)
    {
        detectedLines_.push_back(totalDistanceGroup[i].convertToLine());
    }
}

void ImageProcessor::groupByOrientation(std::vector<LineGroup>& orientationGroup, const std::vector<Line>& lines)
{
    for (size_t i = 0; i < lineCluster_.size(); ++i) {
        groupByOrientation(orientationGroup, lineCluster_[i]);
    }
}

void ImageProcessor::groupByOrientation(std::vector<LineGroup>& group, Line& line)
{
    const double PRECISION_TRESHOLD = 0.9;
    double bestPrecision = 0.0;
    LineGroup*  bestGroup = nullptr;

    bool groupFound = false;
    for (int i = 0; i < group.size(); i++) {
        double precision = std::abs(group[i].getAvgOrientation().dot(line.getOrientation()));
        if (precision > PRECISION_TRESHOLD && precision > bestPrecision)
        {
            bestGroup = &group[i];
            bestPrecision = precision;
            groupFound = true;
        }
    }

    if(groupFound) {
        bestGroup->add(line);
    } else {
        group.push_back(LineGroup(line));
    }
}

void ImageProcessor::groupByIntersection(std::vector<LineGroup>& intersectingGroup,
        const std::vector<LineGroup>& orientationGroup)
{
    for (int i = 0; i < orientationGroup.size(); ++i) {
        const std::vector<Line*>& lines = orientationGroup[i].getLines();
        for (int j = 0; j < lines.size(); ++j)
        {
            groupByIntersection(intersectingGroup, *lines[j]);
        }
    }
}

void ImageProcessor::groupByIntersection(std::vector<LineGroup>& intersectionGroup, Line& line)
{
    bool groupFound = false;
    for (int i = 0; i < intersectionGroup.size(); ++i) {
        cv::Point2f intersection;
        if (intersectionGroup[i].convertToLine().findIntersection(line, intersection)) {
            if (isInsideRect(intersection, cv::Rect(0, 0, 640 ,480))) {
                intersectionGroup[i].add(line);
                groupFound = true;
            }
        }
    }
    if(!groupFound) {
        intersectionGroup.push_back(LineGroup(line));
    }
}

bool ImageProcessor::isInsideRect(const cv::Point2f& point, const cv::Rect& rect)
{
    double xMin = rect.x;
    double xMax = rect.x + rect.width;
    double yMin = rect.y;
    double yMax = rect.y + rect.height;

    return (xMin <= point.x && point.x <= xMax &&
            yMin <= point.y && point.y <= yMax);
}

void ImageProcessor::groupByDistance(std::vector<LineGroup>& distanceGroup, const std::vector<LineGroup>& orientationGroup)
{
    for (int i = 0; i < orientationGroup.size(); ++i) {
        const std::vector<Line*>& lines = orientationGroup[i].getLines();
        for (int j = 0; j < lines.size(); ++j) {
            groupByDistance(distanceGroup, *lines[j]);
        }
    }
}

void ImageProcessor::groupByDistance(std::vector<LineGroup>& distanceGroup, Line& line)
{
    const double ERROR_TRESHOLD = 0.25;
    double bestError = 1.0;
    LineGroup* bestGroup = nullptr;

    bool groupFound = false;
    for (int i = 0; i < distanceGroup.size(); ++i) {
        double dotProduct = line.getOrientation().dot(distanceGroup[i].getAvgOrientation());
        double rectifiedAvgRho = distanceGroup[i].getAvgRho() * dotProduct;
        double error = std::abs((rectifiedAvgRho - line.getRho()) / 800.0);
        if (error < ERROR_TRESHOLD && error < bestError)
        {
            bestGroup = &distanceGroup[i];
            bestError = error;
            groupFound = true;
        }
    }

    if(groupFound) {
        bestGroup->add(line);
    } else {
       distanceGroup.push_back(LineGroup(line));
    }
}

void ImageProcessor::buildLineArray(const std::vector<cv::Vec2f>& lineCluster)
{
    lineCluster_.clear();
    for (int i = 0; i < lineCluster.size(); ++i)
    {
        lineCluster_.push_back(Line(lineCluster[i][0], lineCluster[i][1]));
    }
}


}
