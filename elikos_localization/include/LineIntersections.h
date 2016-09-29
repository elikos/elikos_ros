//
// Created by tonio on 21/03/16.
//

#ifndef IARC7_LOCALIZATION_LINEINTERSECTIONS_H
#define IARC7_LOCALIZATION_LINEINTERSECTIONS_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Dense>

std::vector<cv::Point> findCentroids(std::vector<int> clusters, std::vector<cv::Point> linePoints);

std::vector<cv::Vec2f> getRhoTheta(std::vector<cv::Point> linePoints);

cv::Vec2f getRhoTheta(cv::Point linePoint);

cv::Point findIntersection(cv::Point linePoint1, cv::Point linePoint2);

std::vector<cv::Point> findIntersections(std::vector<int> clusters, std::vector<cv::Point> linePoints);

std::vector<cv::Point> findIntersections(std::vector<int> clusters_x, std::vector<int> clusters_y,
                                         std::vector<cv::Point> linePoints_x, std::vector<cv::Point> linePoints_y);

#endif //IARC7_LOCALIZATION_LINEINTERSECTIONS_H
