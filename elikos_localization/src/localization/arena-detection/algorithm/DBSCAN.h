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
void DBSCAN(const std::vector<Eigen::Vector2f> &dataset, double epsilon, int minPts, std::vector<int>& clusterMemberships);

Eigen::MatrixXd calculateDistMap (const std::vector<cv::Point2f> &dataset);

Eigen::MatrixXd calculateDistMap (const std::vector<cv::Point2f> &dataset1, const std::vector<cv::Point2f> &dataset2);

double calculateDistance (const std::vector<cv::Point2f> &dataset, int i, int j);

double calculateDistance (const std::vector<cv::Point2f> &dataset1, const std::vector<cv::Point2f> &dataset2, int i, int j);

std::vector<int> regionQuery (Eigen::MatrixXd distMap, double epsilon, int i);

}

#endif //DBSCAN_H
