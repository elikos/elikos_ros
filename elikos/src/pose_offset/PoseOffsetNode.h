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
};


#endif //ELIKOS_ROS_POSEOFFSETNODE_H
