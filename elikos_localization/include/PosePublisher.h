//
// Created by Antonio Sanniravong on 15/03/16.
//

#ifndef IARC7_LOCALIZATION_POSEPUBLISHER_H
#define IARC7_LOCALIZATION_POSEPUBLISHER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

class PosePublisher {
public:
    PosePublisher(ros::NodeHandle* nh);
    ~PosePublisher();
    void PublishPose(geometry_msgs::PoseWithCovarianceStamped pose);
    void PublishPose(geometry_msgs::PoseStamped pose);
    void PublishPose2(geometry_msgs::PoseStamped pose);
    void PublishPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
    void PublishPointCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
    void PublishDummyPose();
private:
    ros::NodeHandlePtr nh_;
    ros::Publisher pose_covariance_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher pose_pub2_;
    ros::Publisher point_cloud_pub_;
    ros::Publisher point_cloud_pub2_;
};


#endif //IARC7_LOCALIZATION_POSEPUBLISHER_H
