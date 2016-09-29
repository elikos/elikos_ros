//
// Created by tonio on 09/04/16.
//

#ifndef IARC7_LOCALIZATION_POSITIONESTIMATOR_H
#define IARC7_LOCALIZATION_POSITIONESTIMATOR_H

#include <DBSCAN.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
#include <memory>

struct PointMatchIndex {
    int global;
    int rectified;
    PointMatchIndex(int point1, int point2) {
        global = point1;
        rectified = point2;
    }
};

class PositionEstimator {
public:
    PositionEstimator() { }
    ~PositionEstimator() { }
    geometry_msgs::PoseWithCovarianceStampedPtr ComputePosition(const std::vector<cv::Point2d> &global_intersections,
                                                             const std::vector<cv::Point2d> &rectified_intersections,
                                                             const Eigen::Affine3d &pose);
private:

};


#endif //IARC7_LOCALIZATION_POSITIONESTIMATOR_H
