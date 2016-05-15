
#ifndef TRACKING_TRANSFROMATIONUNIT_H
#define TRACKING_TRANSFROMATIONUNIT_H

#ifndef PI
#define PI 3.14159265
#endif

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <elikos_ros/RobotRawArray.h>
#include <elikos_ros/RobotRaw.h>
#include <string> 
#include <sstream>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
class TransformationUnit
{
	public:
		TransformationUnit();
		//Turret: frame coordinate referencing the camera to robot vector.
		geometry_msgs::PoseArray computeTransformForRobots(elikos_ros::RobotRawArray);
		
	private:
		tf::Quaternion computeTurretRotation(elikos_ros::RobotRaw);
		tf::Transform computeRobotTransform(tf::Transform);
		
        tf::TransformBroadcaster tf_broadcaster_;
        tf::TransformListener tf_listener_;
        tf::Transform camera_;
        
        //TODO: Check the validity of those values
        const double CAMERA_FOV_H = 120 * PI/180.0;
        const double CAMERA_FOV_V = 66 * PI/180.0;
		static const int CAM_HEIGHT = 480;
		static const int CAM_WIDTH = 640;
};

#endif
