#ifndef INTERSECTION_TRANSFORM_H
#define INTERSECTION_TRANSFORM_H

#include <vector>

#include "CameraInfo.h"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>

#include <elikos_ros/Intersection.h>
#include <elikos_ros/IntersectionArray.h>

namespace localization
{

class QuadState;

class IntersectionTransform
{
public:

    IntersectionTransform(const CameraInfo& cameraInfo, const QuadState& state);
    ~IntersectionTransform() = default;

    void transformIntersections(const std::vector<Eigen::Vector2f>& imageIntersections);

private:

    const double GRID_SIDE_LENGTH_M = 1.0;
    const double ALT_ERROR_THRESHOLD = 0.3;

    void updateKDTree(const std::vector<Eigen::Vector2f>& imageIntersections);
    double estimateAltitude(const std::vector<Eigen::Vector2f>& imageIntersections);
    void transformIntersectionXY(const Eigen::Vector2f& imageIntersection, tf::Vector3& intersection, tf::Vector3& offset) const;
    void publishTransformedIntersections(const std::vector<Eigen::Vector2f>& imageIntersections,
                                         const std::vector<tf::Vector3>& TransformedIntersections) const;

    const CameraInfo& cameraInfo_;
    const QuadState& state_;
    ros::Publisher intersectionPub_;

    pcl::KdTreeFLANN<pcl::PointXY> kdTree_;
    pcl::PointCloud<pcl::PointXY>::Ptr pointCloud_;

};

}

#endif // INTERSECTION_TRANSFORM_H