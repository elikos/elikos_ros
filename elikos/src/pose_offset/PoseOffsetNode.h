//
// Created by andre on 15/07/15.
//

#ifndef ELIKOS_ROS_POSEOFFSETNODE_H
#define ELIKOS_ROS_POSEOFFSETNODE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <pose_offset/VoCtrl.h>

class PoseOffsetNode {
public:
    PoseOffsetNode();

    ~PoseOffsetNode() {};

    int main(int argc, char** argv);

protected:
    void LocalPosCallback(const geometry_msgs::PoseStampedConstPtr pose);
    void VOPoseCallback_Cov(const geometry_msgs::PoseWithCovarianceStampedPtr pose);
    void VOPoseCallback(const geometry_msgs::PoseStampedPtr pose);

    ros::NodeHandle _n;

    ros::Subscriber _vo_sub;
    ros::Subscriber _local_pos_sub;
    ros::Publisher _pose_pub;

    geometry_msgs::PoseStamped _local_pose;
    geometry_msgs::PoseStamped _offset_pose;
    geometry_msgs::PoseWithCovarianceStamped _vo_pose;

    bool _publish_covariance;
    bool _recv_covariance;
    bool _local_pos_valid;
    bool _vo_valid;
    std::string _vo_method_s;
    elikos::SupportedVO _vo_method;
    elikos::VoCtrl* _vo_controller;

    int _rate;  //!< Rate in Hz
    int _timeout; //!< Max time [ms] to wait before declaring vo or local_pos timed out

private:
    void setupVoCtrl(std::string method);
};


#endif //ELIKOS_ROS_POSEOFFSETNODE_H
