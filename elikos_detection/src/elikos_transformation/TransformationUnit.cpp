#include "TransformationUnit.h"

TransformationUnit::TransformationUnit()
{
	pub_ = nh_.advertise<elikos_ros::TargetRobotArray>("elikos_target_robot_array",1);
	
	//Parameters
    if (!nh_.getParam("/"+ros::this_node::getName()+"/frame", cameraFrameID_))
  	{
  		cameraFrameID_ = "";
  	}
    if (!nh_.getParam("/"+ros::this_node::getName()+"/cam_fov_h", cam_fov_h_))
  	{
  		cam_fov_h_ = 89;
  	}
	cam_fov_h_ *= PI/180.0;
    if (!nh_.getParam("/"+ros::this_node::getName()+"/cam_fov_v", cam_fov_v_))
  	{
  		cam_fov_v_ = 63;
  	}
	cam_fov_v_ *= PI/180.0;
    if (!nh_.getParam("/"+ros::this_node::getName()+"/cam_height", cam_height_))
  	{
  		cam_height_ = 480;
  	}
    if (!nh_.getParam("/"+ros::this_node::getName()+"/cam_width", cam_width_))
  	{
  		cam_width_ = 640;
  	}
    if (!nh_.getParam("/"+ros::this_node::getName()+"/min_height", min_height_))
  	{
  		min_height_ = 0.1;
  	}
}

geometry_msgs::PoseArray TransformationUnit::computeTransformForRobots(elikos_ros::RobotRawArray robotArray){
    elikos_ros::TargetRobotArray targetArray;

    //results for simulation in rviz
    geometry_msgs::PoseArray results;
    //Stamp the array
    results.header.stamp = ros::Time();
    results.header.frame_id = "elikos_arena_origin";

	//Get the origin to fcu transform
	tf::StampedTransform origin2fcu;
	try {
		tf_listener_.lookupTransform("elikos_arena_origin", "elikos_fcu", ros::Time(0), origin2fcu);
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("TransformationUnit::computeTransformForRobots() exception : %s",ex.what());
		ros::Duration(1.0).sleep();
	}
	//Get the fcu to camera transform
	tf::StampedTransform fcu2camera;
	try {
		tf_listener_.lookupTransform("elikos_fcu", cameraFrameID_, ros::Time(0), fcu2camera);
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("TransformationUnit::computeTransformForRobots() exception : %s",ex.what());
		ros::Duration(1.0).sleep();
	}

	for(auto robot: robotArray.robots){
	
		cv::Point2f position_image;
		position_image.x = robot.point.x;
		position_image.y = robot.point.y;
		cv::Size size_image;
		size_image.width = cam_width_;
		size_image.height = cam_height_;
		geometry_msgs::PoseStamped robotPoseOrigin = transformation_utils::getOrigin2Target(origin2fcu,
                                             fcu2camera,
                                             position_image,
                                             size_image,
                                             cam_fov_h_,
                                             cam_fov_v_);
		/*tf::Quaternion fcu2robot_quat;
		tf::quaternionMsgToTF(robotPoseFcu.pose.orientation, fcu2robot_quat);
		tf::Transform fcu2robot(fcu2robot_quat, tf::Vector3(robotPoseFcu.pose.position.x, robotPoseFcu.pose.position.y, robotPoseFcu.pose.position.y));
		
		tf::Transform origin2robot = origin2fcu * fcu2robot;

		geometry_msgs::PoseStamped robotPoseOrigin;
		robotPoseOrigin.header.stamp = robotPoseFcu.header.stamp;
		robotPoseOrigin.header.frame_id = "elikos_arena_origin";
		robotPoseOrigin.pose.position.x = origin2robot.getOrigin().getX();
		robotPoseOrigin.pose.position.y = origin2robot.getOrigin().getY();
		robotPoseOrigin.pose.position.z = origin2robot.getOrigin().getZ();*/
		results.poses.push_back(robotPoseOrigin.pose);
		//results.poses.push_back(robotPoseFcu.pose);

		elikos_ros::TargetRobot target;
		target.id = robot.id;
		target.color = robot.color;
		target.poseOrigin = robotPoseOrigin;
		//target.poseFcu = robotPoseFcu;
		targetArray.targets.push_back(target);
	}

	if(origin2fcu.getOrigin().z()>min_height_){
		pub_.publish(targetArray);
	}
    return results;
}