//
// Created by Antonio Sanniravong on 15/03/16.
//

#include <PosePublisher.h>

PosePublisher::PosePublisher(ros::NodeHandle* nh) : nh_(nh){
    // Advertise topic
    pose_covariance_pub_ = nh_->advertise<geometry_msgs::PoseWithCovarianceStamped>("/iarc7_localization/pose_covariance", 1);
    pose_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("/iarc7_localization/pose", 1);
    pose_pub2_ = nh_->advertise<geometry_msgs::PoseStamped>("/iarc7_localization/pose2", 1);
    point_cloud_pub_ = nh_->advertise<pcl::PointCloud<pcl::PointXYZ>>("/iarc7_localization/point_cloud", 1);
    point_cloud_pub2_ = nh_->advertise<pcl::PointCloud<pcl::PointXYZ>>("/iarc7_localization/point_cloud2", 1);
}

PosePublisher::~PosePublisher(){}

void PosePublisher::PublishPose(geometry_msgs::PoseWithCovarianceStamped pose_covariance){
    pose_covariance_pub_.publish(pose_covariance);
}

void PosePublisher::PublishPose(geometry_msgs::PoseStamped pose){
    pose_pub_.publish(pose);
}

void PosePublisher::PublishPose2(geometry_msgs::PoseStamped pose){
    pose_pub2_.publish(pose);
}

void PosePublisher::PublishDummyPose(){
    // Publish a dummy pose for testing purposes
    geometry_msgs::PoseWithCovarianceStamped pose;

    // Header
    pose.header.frame_id = "world";
    pose.header.stamp = ros::Time::now();

    // Orientation
    pose.pose.pose.orientation.w = 1.0;
    pose.pose.pose.orientation.x = 0.0;
    pose.pose.pose.orientation.y = 0.0;
    pose.pose.pose.orientation.z = 0.0;

    // Position
    pose.pose.pose.position.x = 0.2;
    pose.pose.pose.position.y = 0.2;
    pose.pose.pose.position.z = 1.0;

    // Covariance
    pose.pose.covariance[0] = 0.5;
    pose.pose.covariance[7] = 0.5;
    pose.pose.covariance[14] = 0.5;

    pose_covariance_pub_.publish(pose);
}

void PosePublisher::PublishPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud) {
    point_cloud_pub_.publish(*point_cloud);
}

void PosePublisher::PublishPointCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud) {
    point_cloud_pub2_.publish(*point_cloud);
}
