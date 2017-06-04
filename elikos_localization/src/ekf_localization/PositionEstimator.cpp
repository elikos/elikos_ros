//
// Created by tonio on 09/04/16.
//

#include "PositionEstimator.h"

geometry_msgs::PoseWithCovarianceStampedPtr PositionEstimator::ComputePosition(const std::vector<cv::Point2d> &global_intersections,
                                                                            const std::vector<cv::Point2d> &rectified_intersections,
                                                                            const Eigen::Affine3d &pose) {
    Eigen::MatrixXd distance_map = DBSCAN::calculateDistMap(global_intersections, rectified_intersections);
    std::vector<PointMatchIndex> point_matches;
    int rectified_index = -1, global_index = -1;
    for (int i = 0; i < rectified_intersections.size(); ++i) {
        distance_map.col(i).minCoeff(&global_index);
        distance_map.row(global_index).minCoeff(&rectified_index);

        if (rectified_index == i && distance_map(global_index, i) < 0.025) {
            // Match
            point_matches.push_back(PointMatchIndex(global_index, rectified_index));
        }
    }

    if (point_matches.size() == 0) {
        return nullptr;
    }

    std::vector<cv::Point2d> errors;
    errors.reserve(point_matches.size());
    for (int i = 0; i < point_matches.size(); ++i) {
        errors.push_back(global_intersections[point_matches[i].global] - rectified_intersections[point_matches[i].rectified]);
    }

    cv::Point2d mean_error;
    for (int i = 0; i < errors.size(); ++i) {
        mean_error += cv::Point2d(errors[i].x / errors.size(), errors[i].y / errors.size());
    }

    std::cout << mean_error << "\n";

    geometry_msgs::PoseWithCovarianceStampedPtr new_pose(new geometry_msgs::PoseWithCovarianceStamped);
    new_pose->header.frame_id = "world";
    new_pose->header.stamp = ros::Time::now();
    new_pose->pose.pose.position.x = pose.translation().x() + mean_error.x;
    new_pose->pose.pose.position.y = pose.translation().y() + mean_error.y;
    new_pose->pose.pose.position.z = /*pose.translation().z()*/ 0.25;
    Eigen::Quaterniond orientation(pose.linear());
    tf::quaternionEigenToMsg(orientation, new_pose->pose.pose.orientation);
    new_pose->pose.pose.orientation.w = 1.0;
    new_pose->pose.covariance[0] = 0.5 / (point_matches.size() + 1);
    new_pose->pose.covariance[7] = 0.5 / (point_matches.size() + 1);
    new_pose->pose.covariance[14] = 0.5;
    return new_pose;
}
