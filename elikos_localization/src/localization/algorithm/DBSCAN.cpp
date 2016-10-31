//
// Created by Antonio Sanniravong
//

#include <iostream>

#include "DBSCAN.h"

#include "Intersection.h"
#include <opencv2/core/core.hpp>

namespace DBSCAN
{

void DBSCAN (const std::vector<cv::Point2f> &dataset, double epsilon, int minPts, std::vector<int>& clusterMemberships) 
{
    if (dataset.empty()) return;

    cv::Mat data(dataset.size(), 2, CV_32F);
    for (int i = 0; i < dataset.size(); ++i) 
    {
        data.at<float>(i, 0) = dataset[i].x;
        data.at<float>(i, 1) = dataset[i].y;
    }

    //cv::KDTree tree(data, false);
    
    Eigen::MatrixXd distanceMap = calculateDistMap(dataset);
    int clusterID = 0;
    std::vector<int> neighborIndexes;
    clusterMemberships.resize(dataset.size(), 0);
    std::vector<bool> visited(dataset.size(), false);
    for (int i = 0; i < dataset.size(); i++) {
        if (visited[i]) continue;
        visited[i] = true;

        std::vector<int> test;
        cv::Mat point(2, 1, CV_32F);
        point.at<float>(0, 0) = dataset[i].x;
        point.at<float>(1, 0) = dataset[i].y;

        bool continuous = point.isContinuous();
        int type = point.type(); 
        int total = point.total();
        int dataT = data.cols;

        //tree.findNearest(point, INT_MAX, epsilon, test);

        neighborIndexes = regionQuery(distanceMap, epsilon, i);
        if (neighborIndexes.size() < minPts) {
            clusterMemberships[i] = -1;
        } else {
            clusterID++;
            clusterMemberships[i] = clusterID;
            for (int j = 0; j < neighborIndexes.size(); ++j) {
                if (!visited[neighborIndexes[j]]) {
                    visited[neighborIndexes[j]] = true;
                    std::vector<int> nestedNeighborIndexes = regionQuery(distanceMap, epsilon, neighborIndexes[j]);
                    if (nestedNeighborIndexes.size() >= minPts){
                        neighborIndexes.insert(neighborIndexes.end(), nestedNeighborIndexes.begin(), nestedNeighborIndexes.end());
                    }
                }
                if (clusterMemberships[neighborIndexes[j]] == 0) {
                    clusterMemberships[neighborIndexes[j]] = clusterID;
                }
            }
        }
    }
}

Eigen::MatrixXd calculateDistMap (const std::vector<cv::Point2f> &dataset) {
    Eigen::MatrixXd distanceMap(dataset.size(), dataset.size());
    for (int i = 0; i < dataset.size(); i++){
        for (int j = i; j < dataset.size(); j++) {
            distanceMap(i, j) = calculateDistance(dataset, i, j);
            distanceMap(j, i) = distanceMap(i, j);
        }
    }
    return distanceMap;
}

Eigen::MatrixXd calculateDistMap (const std::vector<cv::Point2f> &dataset1, const std::vector<cv::Point2f> &dataset2) {
    Eigen::MatrixXd distanceMap(dataset1.size(), dataset2.size());
    for (int i = 0; i < dataset1.size(); i++){
        for (int j = 0; j < dataset2.size(); j++) {
            distanceMap(i, j) = calculateDistance(dataset1, dataset2, i, j);
        }
    }
    return distanceMap;
}

double calculateDistance (const std::vector<cv::Point2f> &dataset, int i, int j) {
    cv::Point2f difference = dataset[i] - dataset[j];
    return ((difference.x * difference.x) + (difference.y * difference.y));
}

double calculateDistance (const std::vector<cv::Point2f> &dataset1, const std::vector<cv::Point2f> &dataset2, int i, int j) {
    cv::Point2f difference = dataset1[i] - dataset2[j];
    return ((difference.x * difference.x) + (difference.y * difference.y));
}

std::vector<int> regionQuery(Eigen::MatrixXd distanceMap, double epsilon, int i) {
    std::vector<int> neighborIndexes;
    for (int j = 0; j < distanceMap.rows(); j++) {
        if (distanceMap(i, j) < epsilon) {
            neighborIndexes.push_back(j);
        }
    }
    return neighborIndexes;
}

}
