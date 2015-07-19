//
// Created by andre on 15/07/15.
//

#ifndef ELIKOS_ROS_POSEOFFSETNODE_H
#define ELIKOS_ROS_POSEOFFSETNODE_H

#include <ros/ros.h>

class PoseOffsetNode {
public:
    PoseOffsetNode();

    ~PoseOffsetNode() {};

    int main();

protected:
    void LocalPosCallback(const geometry_msgs::PoseStampedConstPtr pose);
    void VOPoseCallback(const geometry_msgs::PoseWithCovarianceStampedPtr pose);

    ros::NodeHandle _n;

    ros::Subscriber _vo_sub;
    ros::Subscriber _local_pos_sub;
    ros::Publisher _pose_pub;

    geometry_msgs::PoseStamped _local_pose;
    geometry_msgs::PoseWithCovarianceStamped _vo_pose;

    bool _publish_covariance;
    bool _local_pos_valid;
    bool _vo_valid;

    int _rate;  //!< Rate in Hz
    int _timeout; //!< Max time [ms] to wait before declaring vo or local_pos timed out
};


#endif //ELIKOS_ROS_POSEOFFSETNODE_H
