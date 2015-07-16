//
// Created by andre on 15/07/15.
//

#include <elikos_lib/params_helper.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "PoseOffsetNode.h"

PoseOffsetNode::PoseOffsetNode() :
    _n("/pose_offset"),
    _local_pos_valid(false),
    _vo_valid(false)
{
    //setup parameters
    this->_publish_covariance = elikos::getParam<bool>("publish_covariance", false);

    //setup publications
    if (this->_publish_covariance) {
        _pose_pub = _n.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1);
    } else {
        _pose_pub = _n.advertise<geometry_msgs::PoseStamped>("pose", 1);
    }

    //setup subscriptions and callbacks
    _local_pos_sub = nh.subscribe("mavros/local_position/local", 1, &PoseOffsetNode::LocalPosCallback, this);
    _vo_sub = nh.subscribe("svo/pose", 1, &PoseOffsetNode::vo_pose_callback, this);
}

void PoseOffsetNode::LocalPosCallback(const PoseStampedConstPtr pose)
{
    if(!_local_pos_valid) {
        ROS_WARN("Local pose now valid.");
    }
    _local_pose = *pose;
    _local_pos_valid = true;
}

void PoseOffsetNode::VOPoseCallback(const PoseWithCovarianceStampedPtr pose)
{

}

PoseOffsetNode::main()
{
    ros::Rate r(10);

    // drive the node by callbacks
    ros::spin();
}

