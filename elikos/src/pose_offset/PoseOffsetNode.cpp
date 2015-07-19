//
// Created by andre on 15/07/15.
//

#include <elikos_lib/params_helper.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "PoseOffsetNode.h"
#include "../../../../../../../../../../opt/ros/indigo/include/geometry_msgs/PoseStamped.h"
#include "../../../../../../../../../../opt/ros/indigo/include/std_msgs/Header.h"

PoseOffsetNode::PoseOffsetNode() :
    _n("/pose_offset"),
    _local_pos_valid(false),
    _vo_valid(false)
{
    //setup parameters
    this->_publish_covariance = elikos::getParam<bool>("publish_covariance", false);
    this->_rate = elikos::getParam<int>("rate", 100);
    this->_timeout = elikos::getParam<int>("timeout", 1000);

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
    if(!_vo_valid){
        ROS_WARN("VO pose now valid.");
    }
    _vo_pose = *pose;
    _vo_valid = true;

    //additional processing should follow this line
}

PoseOffsetNode::main()
{
    ros::init(argc, argv, "pose_offset");
    ros::Rate r(this->_rate);   //Not sure what rate to put but i absolutely has to be greater than
                                //what visual odometry is outputting
    ros::Duration timeout(this->_timeout/1000);

    while(ros::ok()){
        if(ros::Time::now() - _local_pose.header.stamp > timeout){
            if(this->_local_pos_valid){
                ROS_ERROR("Local Pose timed out!");
            }
        }

        r.spinOnce();
        r.sleep();
    }
}

