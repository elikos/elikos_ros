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

#include "geometry_msgs/PoseArray.h"

#include <elikos_main/Intersection.h>
#include <elikos_main/IntersectionArray.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

class QuadState;

namespace localization
{

class IntersectionTransform
{
public:

    IntersectionTransform(const CameraInfo& cameraInfo, const QuadState& state);
    ~IntersectionTransform() = default;

    void transformIntersections(const std::vector<Eigen::Vector2f>& imageIntersections,
                                const cv::Mat& perspectiveTransform,
                                cv::Size imageSize);

private:

    const double GRID_SIDE_LENGTH_M = 1.0;
    const double ALT_ERROR_THRESHOLD = 0.3;

    void resetPivot();

    void updateKDTree(const std::vector<Eigen::Vector2f>& imageIntersections);
    double estimateAltitude(const std::vector<Eigen::Vector2f>& imageIntersections);
    void transformIntersectionXY(const Eigen::Vector2f& imageIntersection, tf::Vector3& intersection, tf::Vector3& offset) const;
    void publishTransformedIntersections(const std::vector<cv::Point2f>& imageIntersections,
                                         const geometry_msgs::PoseArray& poseArray);

    void distanceMatrix(const geometry_msgs::PoseArray& lastDetection_,
                        const geometry_msgs::PoseArray& currentDetection,
                        const std::vector<std::vector<float>>& distanceMatrix);

    void estimateQuadState(const geometry_msgs::PoseArray& intersections);

    visualization_msgs::Marker marker_;

    const CameraInfo& cameraInfo_;
    const QuadState& state_;

    ros::Publisher intersectionPub_;
    ros::Publisher debugPub_;
    tf::TransformBroadcaster tfPub_;

    pcl::KdTreeFLANN<pcl::PointXY> imageIntersectionsTree_;
    pcl::PointCloud<pcl::PointXY>::Ptr imageIntersectionsPointCloud_;

    pcl::KdTreeFLANN<pcl::PointXY> lastDetectionTree_;
    pcl::PointCloud<pcl::PointXY>::Ptr lastDetectionPointCloud_;
    geometry_msgs::PoseArray lastDetection_;

    tf::StampedTransform lastState_;

    tf::Vector3 totalTranslation_;
    tf::Vector3 pivot_;
};

}

#endif // INTERSECTION_TRANSFORM_H