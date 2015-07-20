//
// Created by andre on 15/07/15.
//

#include <elikos_lib/params_helper.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "PoseOffsetNode.h"
#include "../../../../../../../../../../opt/ros/indigo/include/geometry_msgs/PoseStamped.h"
#include "../../../../../../../../../../opt/ros/indigo/include/std_msgs/Header.h"
#include "../../../../../../../../../../opt/ros/indigo/include/geometry_msgs/PoseWithCovarianceStamped.h"
#include "../../../../../../../../../../opt/ros/indigo/include/geometry_msgs/PoseWithCovariance.h"
#include "../../../../../../../../../../opt/ros/indigo/include/geometry_msgs/Pose.h"

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
        //save the offset position ever since VO started or came back
        this->_offset_pose = _local_pose;
    }
    this->_vo_pose = *pose;
    this->_vo_valid = true;

    //publish pose offset at the same rate as VO
    if(_local_pos_valid && _vo_valid){
        if (this->_publish_covariance) {
            geometry_msgs::PoseWithCovarianceStamped offset_pose;
            offset_pose.header.stamp = this->_vo_pose.stamp;

            //hopefully what this does is keep the VO covariance matrix
            //and update position and orientation with offsets
            offset_pose.pose = _vo_pose.pose;
            offset_pose.pose.pose.position += this->_offset_pose.pose.position;
            offset_pose.pose.pose.orientation += this->_offset_pose.pose.orientation;

            this->_pose_pub.publish(offset_pose);
        } else {
            geometry_msgs::PoseStamped offset_pose;
            offset_pose.header.stamp = this->_vo_pose.stamp;
            offset_pose.pose.position = this->_vo_pose.pose.pose.position + this->_offset_pose.pose.position;
            offset_pose.pose.orientation = this->_vo_pose.pose.pose.orientation + this->_offset_pose.pose.orientation;

            this->_pose_pub.publish(offset_pose);
        }
    }
}

PoseOffsetNode::main()
{
    ros::init(argc, argv, "pose_offset");
    ros::Rate r(this->_rate);   //Not sure what rate to put but it absolutely has to be greater than
                                //what visual odometry is outputting
    ros::Duration timeout(this->_timeout/1000);

    while(ros::ok()){
        if(ros::Time::now() - _local_pose.header.stamp > timeout){
            if(this->_local_pos_valid){
                ROS_ERROR("Local Pose timed out.");
            }
            this->_local_pos_valid = false;
        }

        if(ros::Time::now() - _vo_pose.header.stamp > timeout) {
            if(this->_vo_valid){
                ROS_ERROR("VO timed out.");
            }
            this->_vo_valid = false;
        }

        r.spinOnce();
        r.sleep();
    }
}

