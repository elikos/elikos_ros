#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Geometry>
#include <iostream>
#include <unordered_map>

#include "GridFitting.h"
#include "LineGroup.h"
#include "DBSCAN.h"

#include "ImageProcessor.h"

namespace localization {

using Vector = Eigen::Vector2f;

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
    start_ = ros::Time::now();

    file_.open("altitude.txt");
    cv::Mat distortedCamera = (cv::Mat_<float>(3,3) << 422.918640,    0.000000,    350.119451,
            0.000000,  423.121112,    236.380265,
            0.000000,    0.000000,      1.000000);

    cv::Mat cameraDistortion = (cv::Mat_<float>(1,5) << -0.321590, 0.089597, 0.001090, -0.000489, 0.000000);

    cv::Mat undistortedCamera = cv::getOptimalNewCameraMatrix(distortedCamera, cameraDistortion, cv::Size(640, 480), 0);

    cv::initUndistortRectifyMap(distortedCamera, cameraDistortion, cv::Mat(), undistortedCamera,
                                cv::Size(640, 480), CV_32FC1, distortionMap1_, distortionMap2_);

    cv::namedWindow("trackers", 1);
    cvCreateTrackbar("C_W", "trackers", &C_W, 1000);
    cvCreateTrackbar("C_H", "trackers", &C_H, 1000);
}

ImageProcessor::~ImageProcessor()
{
    file_.close();
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
    
    double xy = Eigen::Vector2f(imuOrientation_.x(), imuOrientation_.y()).norm();
    double theta = std::abs(atanf(xy / imuOrientation_.z())); 

    double height = 480.0;
    double width = 640.0;

    Eigen::Vector2f src[4] {{0.0, 0.0}, {0.0, height}, { width, height }, { width, 0.0}};
    Eigen::Vector2f dst[4] {{0.0, 0.0}, {0.0, height}, { width, height }, { width, 0.0}};
    
    Eigen::Vector2f leftRotationPoint = { 0.0, height / 2.0 };
    Eigen::Vector2f rightRotationPoint = { width, height / 2.0 };
    Eigen::Vector2f center = { width / 2.0, height / 2.0 };

    Eigen::Vector2f t = transform_.translate(leftRotationPoint, -center);
    Eigen::Vector2f r = transform_.rotate(t, -roll_);
    leftRotationPoint = transform_.translate(r, center);

    t = transform_.translate(rightRotationPoint, -center);
    r = transform_.rotate(t, -roll_);
    rightRotationPoint = transform_.translate(r, center);

    for (int i = 0; i < 4; ++i) {
        Eigen::Vector2f translated = transform_.translate(dst[i], -center);
        Eigen::Vector2f rotated = transform_.rotate(translated, -roll_);
        dst[i] = transform_.translate(rotated, center);
    }

    for (int i = 0; i < 2; ++i) {
        Eigen::Vector2f translated = transform_.translate(dst[i], -leftRotationPoint);
        Eigen::Vector2f rotated = transform_.rotate(translated, -pitch_);
        src[i] = transform_.translate(rotated, leftRotationPoint);
    }

    for (int i = 2; i < 4; ++i) {
        Eigen::Vector2f translated = transform_.translate(dst[i], -rightRotationPoint);
        Eigen::Vector2f rotated = transform_.rotate(translated, pitch_);
        src[i] = transform_.translate(rotated, rightRotationPoint);
    }

    cv::circle(undistorted, { (int)(src[0].x()), (int)(src[0].y()) }, 5, cv::Scalar(0, 0 ,0), -1);
    cv::circle(undistorted, { (int)(src[1].x()), (int)(src[1].y()) }, 5, cv::Scalar(0, 0 ,0), -1);
    cv::circle(undistorted, { (int)(src[2].x()), (int)(src[2].y()) }, 5, cv::Scalar(0, 0 ,0), -1);
    cv::circle(undistorted, { (int)(src[3].x()), (int)(src[3].y()) }, 5, cv::Scalar(0, 0 ,0), -1);

    cv::circle(undistorted, { (int)(dst[0].x()), (int)(dst[0].y()) }, 5, cv::Scalar(0, 200 ,0), -1);
    cv::circle(undistorted, { (int)(dst[1].x()), (int)(dst[1].y()) }, 5, cv::Scalar(0, 200 ,0), -1);
    cv::circle(undistorted, { (int)(dst[2].x()), (int)(dst[2].y()) }, 5, cv::Scalar(0, 200 ,0), -1);
    cv::circle(undistorted, { (int)(dst[3].x()), (int)(dst[3].y()) }, 5, cv::Scalar(0, 200 ,0), -1);

    cv::Point2f tSrc[4], tDst[4];
    for ( int i = 0; i < 4; ++i) {
        tSrc[i] = { src[i].x(), src[i].y() };
        tDst[i] = { dst[i].x(), dst[i].y() };
    }


    cv::Mat perspectiveTransform = cv::getPerspectiveTransform(tSrc, tDst);
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

void ImageProcessor::processImage(const cv::Mat& input, ros::Time stamp)
{
    image_ = input;
    preProcess(input, preProcessed_);

    cv::Mat edges;
    findEdges(preProcessed_, edges);

    lines_ = cv::Mat(input.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    mLines_ = cv::Mat(input.size(), CV_8UC3, cv::Scalar(0, 0, 0));

    findLines(edges, lines_);
    analyzeLineCluster(stamp);

    cv::imshow("input", preProcessed_);
    cv::imshow("mLines", mLines_);
    cv::imshow("lines", lines_);
    cv::waitKey(1);
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
    Vector centroid = line.getCentroid();
    Vector orientation = line.getOrientation();

    cv::Point2f pt1, pt2;
    pt1.x = cvRound(centroid.x() + 1000 * (orientation.x()));
    pt1.y = cvRound(centroid.y() + 1000 * (orientation.y()));
    pt2.x = cvRound(centroid.x() - 1000 * (orientation.x()));
    pt2.y = cvRound(centroid.y() - 1000 * (orientation.y()));
    cv::line(dst, pt1, pt2, color, 1, CV_AA);
}

void ImageProcessor::drawRawLines(cv::Mat &dst, const std::vector<cv::Vec2f> &raw_lines) const {
    for( size_t i = 0; i < raw_lines.size() && i < 100; ++i )
    {
        float rho = raw_lines[i][0];
        float theta = raw_lines[i][1];

        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a * rho, y0 = b * rho;
        pt1.x = cvRound(x0 + 1000 * (-b));
        pt1.y = cvRound(y0 + 1000 * (a));
        pt2.x = cvRound(x0 - 1000 * (-b));
        pt2.y = cvRound(y0 - 1000 * (a));
        line(dst, pt1, pt2, cv::Scalar(100,100,100), 3, CV_AA);
    }
}

void ImageProcessor::drawLineGroup(cv::Mat& dst, const LineGroup& group, const cv::Scalar& color)
{
    std::vector<const Line*> lines = group.getLines();
    for( size_t i = 0; i < lines.size() && i < 100; ++i )
    {
        drawLine(dst, *lines[i], color);
    }
}

void ImageProcessor::findLines(const cv::Mat& edges, cv::Mat& lines)
{
    std::vector<cv::Vec2f> rawLines;
    cv::HoughLines(edges, rawLines, 1, CV_PI / 180, 100, 0, 0 );
    //drawRawLines(lines, rawLines);

    buildLineArray(rawLines);
}

void ImageProcessor::analyzeLineCluster(ros::Time stamp)
{
    if (lineCluster_.size() == 0) 
    {
        return;
    }

    for (int i = 0; i < lineCluster_.size(); ++i) 
    {
        drawLine(mLines_, lineCluster_[i], cv::Scalar(100, 100, 100));
    }

    std::vector<LineGroup> orientationGroup;
    groupByOrientation(orientationGroup, lineCluster_);

    findLineIntersections(orientationGroup);
    std::vector<int> clusterMemberships;
    DBSCAN::DBSCAN(intersections_, 50, 2, clusterMemberships);

    std::vector<Vector> intersections;
    parseClusterMemberships(clusterMemberships, intersections);

    Grid grid;
    bool gridFound = gridFitting_.findBestGridModel(intersections, grid);
    if (gridFound) {
        grid.draw(preProcessed_);
        double height = 423.0 / grid.getDistance();
        std::cout << stamp - start_ << " " << height << std::endl;
        file_ << stamp - start_ << " " << height << std::endl;
    }

    drawIntersection(intersections_, cv::Scalar(150, 150, 0));
    drawIntersection(intersections, cv::Scalar(0, 0, 150));
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

void ImageProcessor::findLineIntersections()
{
    intersections_.clear();
    for (size_t i = 0; i < lineCluster_.size(); ++i) {
        const Line& firstLine = lineCluster_[i];
        for (size_t j = i + 1; j < lineCluster_.size() - 1; j = ((j + 1) % lineCluster_.size())) 
        {
            const Line& otherLine = lineCluster_[j];
            if (std::abs(otherLine.getOrientation().dot(firstLine.getOrientation())) < 0.2)
            {
                continue;
            }
            Vector intersection;
            if (firstLine.findIntersection(lineCluster_[j], intersection)) 
            {
                Eigen::AlignedBox<float, 2> box(Vector(0.0, 0.0), Vector(640.0, 480.0));
                if (box.contains(intersection)) 
                {
                    intersections_.push_back(intersection);
                }
            }
        }
    }
}

void ImageProcessor::drawIntersection(const std::vector<Vector>& intersections, const cv::Scalar& color)
{
    for (int i = 0; i < intersections.size(); ++i) {
        cv::circle(mLines_,  cv::Point2f(intersections[i].x(), intersections[i].y()), 5, color, -1);
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
