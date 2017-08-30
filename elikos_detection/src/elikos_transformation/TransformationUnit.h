
#ifndef TRACKING_TRANSFROMATIONUNIT_H
#define TRACKING_TRANSFROMATIONUNIT_H

#ifndef PI
#define PI 3.14159265
#endif

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <elikos_main/RobotRawArray.h>
#include <elikos_main/RobotRaw.h>
#include <elikos_main/TargetRobotArray.h>
#include <elikos_main/TargetRobot.h>
#include <string>
#include <sstream>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <opencv2/opencv.hpp>
#include "TransformationUtils.h"
class TransformationUnit
{
	public:
		TransformationUnit();
		//Turret: frame coordinate referencing the camera to robot vector.
		geometry_msgs::PoseArray computeTransformForRobots(elikos_main::RobotRawArray);

	private:
		tf::TransformBroadcaster tf_broadcaster_;
		tf::TransformListener tf_listener_;

		ros::NodeHandle nh_;
		ros::Publisher pub_;

	  	std::string cameraFrameID_;

		//TODO: Check the validity of those values
		double cam_fov_h_ = 89 * PI/180.0;
		double cam_fov_v_ = 63 * PI/180.0;
		int cam_height_ = 480;
		int cam_width_ = 640;
		double min_height_ = 0.1;
};

#endif
