//
// Created by tonio on 28/03/16.
//

#ifndef IARC7_LOCALIZATION_GRIDFITTING_H
#define IARC7_LOCALIZATION_GRIDFITTING_H
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/publisher.h>
#include <PositionEstimator.h>
#include <nav_msgs/Odometry.h>

class GridFitting {
public:
    GridFitting();
    ~GridFitting();
    enum Orientation { X, Y };
    std::vector<cv::Vec2f> FitToGrid(std::vector<cv::Vec2f> lines, geometry_msgs::PoseStamped pose, Orientation orientation);
    Eigen::Quaterniond Euler2Quaternion(const double roll, const double pitch, const double yaw);
    const Eigen::Matrix2d &GetPerspectivePoints() const { return perspective_point_coordinates_; }
    const pcl::PointCloud<pcl::PointXYZ>::Ptr GetIntersectionCloud() const { return intersection_cloud_; }
    const pcl::PointCloud<pcl::PointXYZ>::Ptr GetGlobalCloud() const { return global_intersection_cloud_; }
    const std::vector<cv::Point2d> &GetGlobalIntersections() const { return global_intersections_; }
    const std::vector<cv::Point2d> &GetRectifiedIntersections() const { return rectified_intersections_; }
    void SetWorldIMUPose(const nav_msgs::OdometryConstPtr &msg);
    void ComputeGlobalIntersections();
    std::vector<cv::Point3d> ComputeIntersectionDepths(std::vector<cv::Point> intersections);
    std::vector<cv::Point2d> RectifyIntersections(std::vector<cv::Point3d> intersection_depths);
    geometry_msgs::PoseStamped GetPose();
    geometry_msgs::PoseStamped GetPose2();
    geometry_msgs::PoseWithCovarianceStampedPtr GetEstimatedPose();
private:
    PositionEstimator position_estimator_;
    double MinAngleDiff(double a, double b);
    void ComputePerspective(const geometry_msgs::PoseStamped &pose);
    cv::Point3d GetDepthFromPixel(cv::Point pixel);

    double horizontal_fov_ = 88;
    double scaling_factor_ = 1.0;
    Eigen::Array2d perspective_z_;
    Eigen::Affine3d world_imu_pose_;
    Eigen::Array2d image_dimensions_;
    Eigen::Array2d image_dimensions_div2_;
    Eigen::Array2d fov_;
    Eigen::Array2d tan_fov_div2_;
    Eigen::Array2d image_plane_;
    Eigen::Array22d perspective_xy_;
    Eigen::Quaterniond imu_cam_q_;
    Eigen::Quaterniond world_imu_q_;
    Eigen::Quaterniond grid_world_q_;
    Eigen::MatrixXd perspective_points_;
    Eigen::MatrixXd rotated_points_;
    Eigen::MatrixXd world_rotated_points_;
    Eigen::Matrix2d perspective_point_coordinates_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr intersection_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_intersection_cloud_;
    std::vector<cv::Point2d> global_intersections_;
    std::vector<cv::Point2d> rectified_intersections_;
};


#endif //IARC7_LOCALIZATION_GRIDFITTING_H
