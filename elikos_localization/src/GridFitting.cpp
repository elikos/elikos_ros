//
// Created by tonio on 28/03/16.
//

#include "GridFitting.h"

GridFitting::GridFitting()
:   image_dimensions_(640.0, 480.0),
    fov_(0.0, 0.0),
    imu_cam_q_(1.0, 0.0, 0.0, 0.0),
    world_imu_q_(1.0, 0.0, 0.0, 0.0),
    grid_world_q_(1.0, 0.0, 0.0, 0.0),
    perspective_points_(3,2),
    rotated_points_(3,2)
{
    // Set FOV
    fov_(0) = horizontal_fov_ * M_PI / 180.0;
    fov_(1) = (image_dimensions_(1)/image_dimensions_(0)) * fov_(0);

    // Pre-compute constant values
    image_dimensions_div2_ = image_dimensions_ / 2;
    tan_fov_div2_ = (fov_ / 2.0).tan();
    image_plane_ = image_dimensions_div2_.cwiseQuotient(tan_fov_div2_);

    // Set imu to camera rotation
    imu_cam_q_ = Euler2Quaternion(-M_PI_2, 0.0, -M_PI_2);
    grid_world_q_ = Euler2Quaternion(0.0, 0.0, 0.0/*-M_PI/4*/ );

    // Set perspective projection points
    perspective_points_ << 1.0, 0.0,
                           0.0, 1.0,
                           0.0, 0.0;

    // Compute all the grid intersections
    ComputeGlobalIntersections();
}

GridFitting::~GridFitting() {

}

std::vector<cv::Vec2f> GridFitting::FitToGrid(std::vector<cv::Vec2f> lines, geometry_msgs::PoseStamped pose, Orientation orientation) {
    ComputePerspective(pose);
    std::vector<cv::Vec2f> pruned_lines;
    pruned_lines.reserve(lines.size());

    int point_index_;
    switch (orientation) {
        case X: point_index_ = 0;
            break;
        case Y: point_index_ = 1;
    }

    for (int i = 0; i < lines.size() && i < 80; ++i) {
        float rho = lines[i][0], theta = lines[i][1];
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        double optimal_angle = atan2f(perspective_point_coordinates_(point_index_,1) - y0, perspective_point_coordinates_(point_index_,0) - x0);
        if (MinAngleDiff(theta - M_PI_2, optimal_angle) < 15 * M_PI / 180.0 ||
            MinAngleDiff(theta + M_PI_2, optimal_angle) < 15 * M_PI / 180.0 ) {
            pruned_lines.push_back(lines[i]);
        }

    }
    return pruned_lines;

}

void GridFitting::ComputePerspective(const geometry_msgs::PoseStamped &pose) {
    tf::quaternionMsgToEigen(pose.pose.orientation, world_imu_q_);
    rotated_points_ =  (/*grid_world_q_ **/world_imu_q_ * imu_cam_q_).toRotationMatrix().inverse() * perspective_points_;

    perspective_xy_ = rotated_points_.block(0, 0, 2, 2).array();
    perspective_z_ = rotated_points_.block(2, 0, 1, 2).array().transpose();
//    std::cout << perspective_xy_(0, 0) << '\t' << perspective_xy_(1, 0) << '\t' << perspective_z_(0) << '\n';
    perspective_point_coordinates_(0, 0) = image_dimensions_div2_(0) + image_plane_(0) * perspective_xy_(0, 0) / perspective_z_(0) ;
    perspective_point_coordinates_(0, 1) = image_dimensions_div2_(1) + image_plane_(1) * perspective_xy_(1, 0) / perspective_z_(0) ;
    perspective_point_coordinates_(1, 0) = image_dimensions_div2_(0) + image_plane_(0) * perspective_xy_(0, 1) / perspective_z_(1) ;
    perspective_point_coordinates_(1, 1) = image_dimensions_div2_(1) + image_plane_(1) * perspective_xy_(1, 1) / perspective_z_(1) ;
}

Eigen::Quaterniond GridFitting::Euler2Quaternion(const double roll, const double pitch, const double yaw) {
    Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;
    return q;
}

double GridFitting::MinAngleDiff(double a, double b) {
    return fabs(atan2(sin(a-b), cos(a-b)));
}

void GridFitting::ComputeGlobalIntersections() {
    double cell_size = 0.10;
    int width = 6;
    int height = 6;
    std::vector<cv::Point2d> global_intersections;
    global_intersections.reserve(width * height);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    point_cloud->header.frame_id = "world";
//    point_cloud->header.stamp = ros::Time::now().toNSec();
    pcl_conversions::toPCL(ros::Time::now(), point_cloud->header.stamp);
    point_cloud->width = width;
    point_cloud->height = height;

    for (int i = 0; i < point_cloud->height * point_cloud->width; ++i) {
        point_cloud->points.push_back(pcl::PointXYZ(cell_size * (i / width - width / 2) * scaling_factor_,
                                                    cell_size * (i % height - height / 2) * scaling_factor_,
                                                    0.0));
        global_intersections.push_back(cv::Point2d(cell_size * (i / width - width / 2),
                                                   cell_size * (i % height - height / 2)));
    }
    global_intersections_ = global_intersections;
    global_intersection_cloud_ = point_cloud;
}

cv::Point3d GridFitting::GetDepthFromPixel(cv::Point pixel) {
    Eigen::Vector3d gravity_vector(0.0, 0.0, -1.0);
    double roll = -((pixel.y - image_dimensions_div2_(1)) / image_dimensions_(1)) * fov_(1);
    double pitch = ((pixel.x - image_dimensions_div2_(0)) / image_dimensions_(0)) * fov_(0);
    Eigen::Quaterniond pixel_rotation_from_front = Euler2Quaternion(roll, pitch, 0.0);
    Eigen::Vector3d pixel_vector((grid_world_q_ * world_imu_q_ * imu_cam_q_ * pixel_rotation_from_front) * Eigen::Vector3d::UnitZ());
    Eigen::Vector3d cam_front((grid_world_q_ * world_imu_q_ * imu_cam_q_) * Eigen::Vector3d::UnitZ());
    double pixel_angle_from_gravity = acos(pixel_vector.dot(gravity_vector));
    double cam_front_angle_from_gravity = acos(cam_front.dot(gravity_vector));
    double pixel_angle_from_cam_front = acos(cam_front.dot(pixel_vector));
    double pixel_angle_from_x = atan2(pixel.y - image_dimensions_div2_(1), pixel.x - image_dimensions_div2_(0));
//    double height = (grid_world_q_ * world_imu_q_ * world_imu_pose_.translation()).z();
    double height = 0.25;
    double depth = height / cos(pixel_angle_from_gravity);
    double xy = depth * sin(pixel_angle_from_cam_front);
    double x = xy * cos(pixel_angle_from_x);
    double y = xy * sin(pixel_angle_from_x);
    double z = depth * cos(cam_front_angle_from_gravity - pixel_angle_from_gravity);
    return cv::Point3d(x, y, z);
}

std::vector<cv::Point3d> GridFitting::ComputeIntersectionDepths(std::vector<cv::Point> intersections) {
    std::vector<cv::Point3d> intersections_with_depths;
    intersections_with_depths.reserve(intersections.size());

    for (int i = 0; i < intersections.size(); ++i) {
        intersections_with_depths.push_back(GetDepthFromPixel(intersections[i]));
    }
    return intersections_with_depths;
}

std::vector<cv::Point2d> GridFitting::RectifyIntersections(std::vector<cv::Point3d> intersection_depths) {
    std::vector<cv::Point2d> rectified_intersections(intersection_depths.size());
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    point_cloud->header.frame_id = "world";
//    point_cloud->header.stamp = ros::Time::now().toNSec();
    pcl_conversions::toPCL(ros::Time::now(), point_cloud->header.stamp);
    point_cloud->height = 1;
    point_cloud->width = intersection_depths.size();

    for (int i = 0; i < intersection_depths.size(); ++i) {
        Eigen::Vector3d intersection_point(intersection_depths[i].x, intersection_depths[i].y, intersection_depths[i].z);
        intersection_point = (world_imu_q_ * imu_cam_q_).toRotationMatrix() * intersection_point + world_imu_pose_.translation();
//        intersection_point = world_imu_q_.inverse() * (intersection_point /*- world_imu_pose_.translation()*/);
//        intersection_point = intersection_point - world_imu_pose_.translation();
//        intersection_point = world_imu_pose_.inverse() * intersection_point;
        cv::Point intersection_xy(cvRound(image_dimensions_div2_(0) + intersection_point.x()), cvRound(image_dimensions_div2_(1) + intersection_point.y()));
        rectified_intersections[i] = cv::Point2d(intersection_point.x(), intersection_point.y());
        point_cloud->points.push_back(pcl::PointXYZ(intersection_point.x() * scaling_factor_, intersection_point.y() * scaling_factor_, 0.0/*intersection_point.z()*/));
//        point_cloud->points.push_back(pcl::PointXYZ(intersection_depths[i].x, intersection_depths[i].y, intersection_depths[i].z));
    }
    intersection_cloud_ = point_cloud;
    rectified_intersections_ = rectified_intersections;
    return rectified_intersections;
}

geometry_msgs::PoseStamped GridFitting::GetPose() {
    geometry_msgs::PoseStamped pose;
    tf::quaternionEigenToMsg(world_imu_q_ * imu_cam_q_ /** grid_world_q_*/, pose.pose.orientation);
    pose.pose.position.x = 0.0; //world_imu_pose_.translation().x();
    pose.pose.position.y = 0.0; //world_imu_pose_.translation().y();
    pose.pose.position.z = 0.2 * scaling_factor_; //world_imu_pose_.translation().z();
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    return pose;
}

geometry_msgs::PoseStamped GridFitting::GetPose2() {
    cv::Point pixel(0, 480);
    double roll = -((pixel.y - image_dimensions_div2_(1)) / image_dimensions_(1)) * fov_(1);
    double pitch = ((pixel.x - image_dimensions_div2_(0)) / image_dimensions_(0)) * fov_(0);
    Eigen::Quaterniond pixel_rotation_from_front = Euler2Quaternion(roll, pitch, 0.0);
    Eigen::Vector3d cam_front((grid_world_q_ * world_imu_q_ * imu_cam_q_ * pixel_rotation_from_front) * Eigen::Vector3d::UnitZ());

    Eigen::Vector3d gravity_vector(0.0, 0.0, -1.0);
    Eigen::Vector3d rotation_axis = cam_front.cross(gravity_vector);

    geometry_msgs::PoseStamped pose;
    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.position.x = cam_front.x();
    pose.pose.position.y = cam_front.y();
    pose.pose.position.z = cam_front.z();
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    return pose;

}

geometry_msgs::PoseWithCovarianceStampedPtr GridFitting::GetEstimatedPose() {
    return position_estimator_.ComputePosition(global_intersections_, rectified_intersections_, world_imu_pose_);
}

void GridFitting::SetWorldIMUPose(const nav_msgs::OdometryConstPtr &msg) {
    tf::poseMsgToEigen(msg->pose.pose, world_imu_pose_);
}
