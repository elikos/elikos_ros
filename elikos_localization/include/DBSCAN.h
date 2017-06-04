//
// Created by Antonio Sanniravong
//

#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Dense>

namespace DBSCAN
{
std::vector<int> DBSCAN(const std::vector<cv::Point> &dataset, double epsilon, int minPts);

Eigen::MatrixXd calculateDistMap (const std::vector<cv::Point> &dataset);

Eigen::MatrixXd calculateDistMap (const std::vector<cv::Point2d> &dataset1, const std::vector<cv::Point2d> &dataset2);

double calculateDistance (const std::vector<cv::Point> &dataset, int i, int j);

double calculateDistance (const std::vector<cv::Point2d> &dataset1, const std::vector<cv::Point2d> &dataset2, int i, int j);

std::vector<int> regionQuery (Eigen::MatrixXd distMap, double epsilon, int i);
}

#endif //DBSCAN_H
