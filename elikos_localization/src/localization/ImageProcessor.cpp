//
// Created by olivier on 10/2/16.
//

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include "Intersection.h"
#include <unordered_map>

#include "LineGroup.h"
#include "DBSCAN.h"

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
    srand(time(NULL));
    // Init undistortion map
    cv::Mat distortedCamera = (cv::Mat_<float>(3,3) << 422.918640,    0.000000,    350.119451,
            0.000000,  423.121112,    236.380265,
            0.000000,    0.000000,      1.000000);

    cv::Mat cameraDistortion = (cv::Mat_<float>(1,5) << -0.321590, 0.089597, 0.001090, -0.000489, 0.000000);

    cv::Mat undistortedCamera = cv::getOptimalNewCameraMatrix(distortedCamera, cameraDistortion, cv::Size(640, 480), 0);

    cv::initUndistortRectifyMap(distortedCamera, cameraDistortion, cv::Mat(), undistortedCamera,
                                cv::Size(640, 480), CV_32FC1, distortionMap1_, distortionMap2_);

    cv::namedWindow("trackers", 1);
    cvCreateTrackbar("p1x", "trackers", &corners[0].x, 640);
    cvCreateTrackbar("p1y", "trackers", &corners[0].y, 480);
    cvCreateTrackbar("p2x", "trackers", &corners[1].x, 640);
    cvCreateTrackbar("p2y", "trackers", &corners[1].y, 480);
    cvCreateTrackbar("p3x", "trackers", &corners[2].x, 640);
    cvCreateTrackbar("p3y", "trackers", &corners[2].y, 480);
    cvCreateTrackbar("p4x", "trackers", &corners[3].x, 640);
    cvCreateTrackbar("p4y", "trackers", &corners[3].y, 480);
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

    cv::Point2f corners2[4];
    double distance = std::abs(corners[0].x - corners[3].x);
    for (int i = 0; i < 4; ++i) {
        corners2[i] = corners[0];
    }
    corners2[1].y += distance;
    corners2[3].x += distance;
    corners2[2].x += distance;
    corners2[2].y += distance;

    cv::Point2f src[4] {{0.0, 0.0}, {1.0, 1.0}, {2.0, 1.0}, {3.0, 0.0}};
    cv::Point2f dst[4] {{0.0, 0.0}, {0.0, 3.0}, {3.0, 3.0}, {3.0, 0.0}};

    /*
    for (int i = 0; i < 4; i++)
    {
        src[i] = corners[i];
        dst[i] = corners2[i];
    }

    cv::circle(undistorted, src[0], 5, cv::Scalar(0, 0 ,0), -1);
    cv::circle(undistorted, src[1], 5, cv::Scalar(0, 0 ,0), -1);
    cv::circle(undistorted, src[2], 5, cv::Scalar(0, 0 ,0), -1);
    cv::circle(undistorted, src[3], 5, cv::Scalar(0, 0 ,0), -1);

    cv::circle(undistorted, dst[0], 5, cv::Scalar(0, 200 ,0), -1);
    cv::circle(undistorted, dst[1], 5, cv::Scalar(0, 200 ,0), -1);
    cv::circle(undistorted, dst[2], 5, cv::Scalar(0, 200 ,0), -1);
    cv::circle(undistorted, dst[3], 5, cv::Scalar(0, 200 ,0), -1);

    */

    cv::Mat perspectiveTransform = cv::getPerspectiveTransform(src, dst);
    cv::Mat perspective;

    cv::warpPerspective(undistorted, perspective, perspectiveTransform, undistorted.size());
    cv::imshow("undistorted", undistorted);
    cv::imshow("perspective", perspective);

    cv::Mat eroded;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    cv::erode(blured, eroded, element, cv::Point(0), 8);

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

    lines_ = cv::Mat(input.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    lineGroups_ = cv::Mat(input.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    mLines_ = cv::Mat(input.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    intersectionGroup_ = cv::Mat(input.size(), CV_8UC3, cv::Scalar(0, 0, 0));

    findLines(edges, lines_);

    cv::imshow("input", preProcessed);
    cv::imshow("mLines", mLines_);
    cv::imshow("lines", lines_);
    cv::waitKey(30);
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
    cv::Vec2d orientation = line.getOrientation();

    cv::Point2f pt1, pt2;
    pt1.x = cvRound(centroid.x + 1000*(-orientation[0]));
    pt1.y = cvRound(centroid.y + 1000*(orientation[1]));
    pt2.x = cvRound(centroid.x - 1000*(-orientation[0]));
    pt2.y = cvRound(centroid.y - 1000*(orientation[1]));
    cv::line(dst, pt1, pt2, color, 1, CV_AA);
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

    for (int i = 0; i < lineCluster_.size(); ++i) {
        drawLine(mLines_, lineCluster_[i], cv::Scalar(100, 100, 100));
    }

    std::vector<LineGroup> orientationGroup;
    groupByOrientation(orientationGroup, lineCluster_);

    findLineIntersections(orientationGroup);
    std::vector<int> clusterMemberships;
    DBSCAN::DBSCAN(intersections_, 2000, 2, clusterMemberships);

    std::vector<cv::Point2f> intersections;
    parseClusterMemberships(clusterMemberships, intersections);

    drawIntersection(intersections_, cv::Scalar(150, 150, 0));
    drawIntersection(intersections, cv::Scalar(0, 0, 150));

    /*

    for (int i = 0; i < orientationGroup.size(); ++i)
    {
        int r = (i % 3 == 0) ? 100 : 0;
        int g = (i % 3 == 1) ? 100 : 0;
        int b = (i % 3 == 2) ? 100 : 0;

        drawLineGroup(lineGroups_, orientationGroup[i], cv::Scalar(r, g, b));
    }
     */
}

void ImageProcessor::parseClusterMemberships(const std::vector<int>& clusterMemberships, std::vector<cv::Point2f>& intersections)
{
    if (clusterMemberships.size() != intersections_.size()) return;

    std::unordered_map<int, std::pair<cv::Point2f, int>> groups;

    for (int i = 0; i < clusterMemberships.size(); ++i) {
        int groupId = clusterMemberships[i];
        std::unordered_map<int, std::pair<cv::Point2f, int>>::iterator it = groups.find(groupId);
        if (it != groups.end()) {
            it->second.first *= it->second.second;
            it->second.first += intersections_[i];
            it->second.second++;
            it->second.first.x /= it->second.second;
            it->second.first.y /= it->second.second;
        } else {
            groups.insert({groupId, {intersections_[i], 1}});
        }
    }

    std::unordered_map<int, std::pair<cv::Point2f, int>>::iterator it;
    for(it = groups.begin(); it != groups.end(); it++) {
        intersections.push_back(it->second.first);
    }
}

void ImageProcessor::findLineIntersections(const std::vector<LineGroup>& orientationGroups)
{
    intersections_.clear();
    for (size_t i = 0; i < orientationGroups.size(); ++i) {
        const LineGroup& firstGroup = orientationGroups[i];
        for (size_t j = (i + 1) % orientationGroups.size(); j < orientationGroups.size() - 1; j++) {
            const LineGroup& otherGroup = orientationGroups[j];
            findLineIntersections(firstGroup, otherGroup);
        }
    }
}

void ImageProcessor::findLineIntersections(const LineGroup& firstGroup, const LineGroup otherGroup)
{
    const std::vector<Line*> firstLines = firstGroup.getLines();
    const std::vector<Line*> otherLines = otherGroup.getLines();

    for (int i = 0; i < firstLines.size(); ++i)
    {
        for (int j = 0; j < otherLines.size(); j++)
        {
            cv::Point2f intersection;
            if (firstLines[i]->findIntersection(*otherLines[j], intersection)) {
                if (intersection.inside({0, 0, 640, 480})) {
                    intersections_.push_back(intersection);
                }
            }
        }
    }
}

void ImageProcessor::findLineIntersections()
{
    intersections_.clear();
    for (size_t i = 0; i < lineCluster_.size(); ++i) {
        const Line& line = lineCluster_[i];
        for (size_t j = i + 1; j < lineCluster_.size() - 1; j = ((j + 1) % lineCluster_.size())) {
            cv::Point2f intersection;
            if (line.findIntersection(lineCluster_[j], intersection)) {
                if (intersection.inside({0, 0, 640, 480})) {
                    intersections_.push_back(intersection);
                }
            }
        }
    }
}

void ImageProcessor::drawIntersection(const std::vector<cv::Point2f>& intersections, const cv::Scalar& color)
{
    for (int i = 0; i < intersections.size(); ++i) {
        cv::circle(mLines_, intersections[i], 5, color, -1 );
    }
}

void ImageProcessor::drawIntersection(const std::vector<Intersection>& intersections, const cv::Scalar& color)
{
    for (int i = 0; i < intersections.size(); ++i) {
        cv::circle(mLines_, cv::Point2f(intersections[i].x, intersections[i].y), 5, color, -1 );
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

void ImageProcessor::fitLinesInGrid(std::vector<LineGroup>& fittingGroup, std::vector<LineGroup>& orientationGroup)
{
    for (int i = 0; i < orientationGroup.size(); ++i) {
        const std::vector<Line*>& lines = orientationGroup[i].getLines();
        for (int j = 0; j < lines.size(); ++j) {
            fitLinesInGrid(fittingGroup, orientationGroup[i], *lines[j]);
        }
    }
}

double resolveGridFittingAccuracy(const Line& line, const LineGroup& group, double distanceThreshold) {

    const std::vector<Line*>& lines = group.getLines();
    for (int i = 0; i < lines.size(); ++i) {
        double error = std::abs((line.getRho() - lines[i]->getRho()) / 800.0);
    }
}

void ImageProcessor::fitLinesInGrid(std::vector<LineGroup>& fittingGroup, LineGroup group, Line& line)
{
    bool lineFits = false;
    double bestGridFittingAccuracy = 0.0;

    const std::vector<Line*>& lines = group.getLines();
    for (int i = 0; i < lines.size(); ++i) {
        int randomLine = rand() % lines.size();
        double distance = std::abs(lines[randomLine]->getRho() - lines[i]->getRho());
        double gridFittingAccuracy = resolveGridFittingAccuracy(*lines[i], group, distance );
        if (gridFittingAccuracy > bestGridFittingAccuracy) {

        }
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
        std::vector<LineGroup> tmpGroup;
        for (int j = 0; j < lines.size(); ++j) {
            groupByDistance(tmpGroup, *lines[j]);
        }
        distanceGroup.insert(distanceGroup.end(), tmpGroup.begin(), tmpGroup.end());
    }
}

void ImageProcessor::groupByDistance(std::vector<LineGroup>& distanceGroup, Line& line)
{
    const double ERROR_TRESHOLD = 0.40;
    double bestError = 1.0;
    LineGroup* bestGroup = nullptr;

    bool groupFound = false;
    for (int i = 0; i < distanceGroup.size(); ++i) {
        double error = std::abs((distanceGroup[i].getAvgRho() - line.getRho()) / 800.0);

        bool isIntersectingOnScreen = false;
        cv::Point2f intersection;
        if (distanceGroup[i].convertToLine().findIntersection(line, intersection)) {
            isIntersectingOnScreen = isInsideRect(intersection, cv::Rect(0, 0, 640, 480));
        }

        if (isIntersectingOnScreen) {
            bestGroup = &distanceGroup[i];
            break;
        } else if (error < ERROR_TRESHOLD && error < bestError) {
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


double resolveLineProximity(const Line& line, const Line& otherLine, double errorThreshold)
{
    double dotProduct = line.getOrientation().dot(otherLine.getOrientation());
    double rectifiedRho = otherLine.getRho() * dotProduct;
    return std::abs((rectifiedRho - line.getRho()) / 800.0);
}

}
