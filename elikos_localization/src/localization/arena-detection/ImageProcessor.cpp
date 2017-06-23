#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Geometry>
#include <iostream>
#include <unordered_map>

#include "LineGroup.h"
#include "DBSCAN.h"

#include "ImageProcessor.h"
namespace localization {

using Vector = Eigen::Vector2f;

ImageProcessor::ImageProcessor(const CameraInfo& cameraInfo, const QuadState& state)
    : cameraInfo_(cameraInfo), state_(state), preProcessing_(cameraInfo, state), intersectionTransform_(cameraInfo, state)
{
    srand(time(NULL));
}

ImageProcessor::~ImageProcessor()
{
}

void ImageProcessor::processImage(cv::Mat& input, cv::Mat& result)
{
    cv::Mat preProcessed;
    preProcessing_.preProcessImage(input, preProcessed);

    cv::Mat edges;
    findEdges(preProcessed, edges);

    findLines(edges);

    result = cv::Mat(input.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    analyzeLineCluster(result);
}
void ImageProcessor::findEdges(const cv::Mat& src, cv::Mat& edges)
{
    cv::Canny(src, edges, 1, 30, 3);
}


void ImageProcessor::drawLine(cv::Mat& dst, const Line& line, const cv::Scalar& color) const
{
    Vector centroid = line.getCentroid();
    Vector orientation = line.getOrientation();

    cv::Point2f pt1, pt2;
    pt1.x = cvRound(centroid.x() + 1000 * (orientation.x()));
    pt1.y = cvRound(centroid.y() + 1000 * (orientation.y()));
    pt2.x = cvRound(centroid.x() - 1000 * (orientation.x()));
    pt2.y = cvRound(centroid.y() - 1000 * (orientation.y()));
    cv::line(dst, pt1, pt2, color, 1, CV_AA);
    cv::circle(dst, cv::Point2f(centroid.x(), centroid.y()), 5, cv::Scalar(150, 0, 150), -1);
}

void ImageProcessor::findLines(const cv::Mat& edges)
{
    std::vector<cv::Vec2f> rawLines;
    cv::HoughLines(edges, rawLines, 1, CV_PI / 180, 100, 0, 0 );

    buildLineArray(rawLines);
}

void ImageProcessor::analyzeLineCluster(cv::Mat& debug)
{
    if (lineCluster_.size() == 0)
    {
        return;
    }

    for (int i = 0; i < lineCluster_.size(); ++i)
    {
        drawLine(debug, lineCluster_[i], cv::Scalar(100, 100, 100));
    }

    lineDetection_.filterLineCluster(lineCluster_);
    const std::vector<Line>& filteredLines = lineDetection_.getFilteredLines();
    for (int i = 0; i < filteredLines.size(); ++i)
    {
        drawLine(debug, filteredLines[i], cv::Scalar(150, 0, 150));
    }

    std::vector<LineGroup> orientationGroup;
    groupByOrientation(orientationGroup, lineCluster_);


    findLineIntersections(orientationGroup);
    std::vector<int> clusterMemberships;
    int radius = 0.5 * cameraInfo_.focalLength / state_.getOrigin2Fcu().getOrigin().z();
    DBSCAN::DBSCAN(intersections_, radius, 2, clusterMemberships);

    std::vector<Vector> intersections;
    parseClusterMemberships(clusterMemberships, intersections);

    drawIntersection(intersections_, cv::Scalar(150, 150, 0), debug);
    drawIntersection(intersections, cv::Scalar(0, 0, 150), debug);

    intersectionTransform_.transformIntersections(intersections);
}

void ImageProcessor::parseClusterMemberships(const std::vector<int>& clusterMemberships, std::vector<Vector>& intersections)
{
    if (clusterMemberships.size() != intersections_.size()) return;

    std::unordered_map<int, std::pair<Vector, int>> groups;

    for (int i = 0; i < clusterMemberships.size(); ++i) {
        int groupId = clusterMemberships[i];
        std::unordered_map<int, std::pair<Vector, int>>::iterator it = groups.find(groupId);
        if (it != groups.end()) {
            it->second.first *= it->second.second;
            it->second.first += intersections_[i];
            it->second.second++;
            it->second.first.x() /= it->second.second;
            it->second.first.y() /= it->second.second;
        } else {
            groups.insert({groupId, {intersections_[i], 1}});
        }
    }

    std::unordered_map<int, std::pair<Vector, int>>::iterator it;
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
    const std::vector<const Line*> firstLines = firstGroup.getLines();
    const std::vector<const Line*> otherLines = otherGroup.getLines();

    for (int i = 0; i < firstLines.size(); ++i)
    {
        const Line* firstLine = firstLines[i];
        for (int j = 0; j < otherLines.size(); j++)
        {
            const Line* otherLine = otherLines[j];
            if (std::abs(otherLine->getOrientation().dot(firstLine->getOrientation())) > 0.8)
            {
                continue;
            }
            Vector intersection;
            if (firstLines[i]->findIntersection(*otherLines[j], intersection)) {
                Eigen::AlignedBox<float, 2> box(Vector(0.0, 0.0), Vector(640.0, 480.0));
                if (box.contains(intersection)) {
                    intersections_.push_back(intersection);
                }
            }
        }
    }
}

void ImageProcessor::drawIntersection(const std::vector<Vector>& intersections, const cv::Scalar& color, cv::Mat& dst)
{
    for (int i = 0; i < intersections.size(); ++i) {
        cv::circle(dst,  cv::Point2f(intersections[i].x(), intersections[i].y()), 5, color, -1);
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

void ImageProcessor::buildLineArray(const std::vector<cv::Vec2f>& lineCluster)
{
    Eigen::AlignedBox<float, 2> frame(Vector(0.0, 0.0), Vector(640.0, 480.0));
    lineCluster_.clear();
    for (int i = 0; i < lineCluster.size(); ++i)
    {
        lineCluster_.push_back(Line(lineCluster[i][0], lineCluster[i][1], frame));
    }
}

}
