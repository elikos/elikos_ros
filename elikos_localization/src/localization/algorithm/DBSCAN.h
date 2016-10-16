//
// Created by Antonio Sanniravong
//

#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Dense>
#include <Intersection.h>

namespace DBSCAN
{
void DBSCAN(const std::vector<cv::Point2d> &dataset, double epsilon, int minPts, std::vector<int>& clusterMemberships);

Eigen::MatrixXd calculateDistMap (const std::vector<cv::Point2d> &dataset);

Eigen::MatrixXd calculateDistMap (const std::vector<cv::Point2d> &dataset1, const std::vector<cv::Point2d> &dataset2);

double calculateDistance (const std::vector<cv::Point2d> &dataset, int i, int j);

double calculateDistance (const std::vector<cv::Point2d> &dataset1, const std::vector<cv::Point2d> &dataset2, int i, int j);

std::vector<int> regionQuery (Eigen::MatrixXd distMap, double epsilon, int i);

void parseClusterMemberShips(const std::vector<int>& clusterMemberShips,
                                                 const std::vector<cv::Point2d>& intersections,
                                                 std::vector<localization::Intersection>& parsedClusters);

}

#endif //DBSCAN_H
