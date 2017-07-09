#include "TransformationUnit.h"
#include "TransformationUtils.h"

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

geometry_msgs::PoseArray TransformationUnit::computeTransformForRobots(const elikos_ros::RobotRawArray& robotArray){
    elikos_ros::TargetRobotArray targetArray;

    //results for simulation in rviz
    geometry_msgs::PoseArray results;
    //Stamp the array
    results.header.stamp = ros::Time();
    results.header.frame_id = "elikos_arena_origin";

	//Get the origin to camera transform
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
		cv::Point2f point(robot.point.x, robot.point.y);
		cv::Size camSize(cam_width_, cam_height_);
		//TODO: Does this return pose in the FCU or origin reference frame?
        geometry_msgs::PoseStamped poseFcu = transformation_utils::getFcu2Target(
				origin2fcu,
				fcu2camera,
				point,
				camSize,
				cam_fov_h_,
				cam_fov_v_
		);
		geometry_msgs::PoseStamped poseOrigin;
		//Transform pose to origin frame
		tf_listener_.transformPose("elikos_arena_origin", poseFcu, poseOrigin);

		//Create target and add it to target array
		elikos_ros::TargetRobot target;
		target.color = robot.color;
		target.id = robot.id;
		target.poseFcu = poseFcu;
		target.poseOrigin = poseOrigin;

		targetArray.targets.push_back(target);

		//Add to results array
		results.poses.push_back(target.poseOrigin.pose);
    }

	if(origin2fcu.getOrigin().z()>min_height_){
		pub_.publish(targetArray);
		oldArray_ = targetArray;
	}
  return results;
}
tf::Transform TransformationUnit::computeRobotTransform(const tf::Transform& origin2turret){
	// Get the smallest angle between the turret and the z axis
	// - First get the vector pointing towards the z axis of the turret
	tf::Vector3 vect_z = tf::quatRotate(origin2turret.getRotation(), tf::Vector3(0, 0, 1));
	// - Then find it's angle with the camera's resting position (-z)
	tf::Vector3 zAxis(0, 0, -1);
	double zAxis_turret_angle = zAxis.angle(vect_z);

	// Initiallization on the robot frame coordinate
	tf::Transform robotFrame = tf::Transform::getIdentity();
	robotFrame.setOrigin(tf::Vector3(0, 0, 0));

	// Get distance from turret to target (using angle and altitude) 
	double altitude = origin2turret.getOrigin().getZ();
	double distance_from_robot = altitude / cos(zAxis_turret_angle);
	robotFrame.setOrigin(tf::Vector3(0, 0, distance_from_robot));
	//Set the orientation
	//the robots are on the ground, so we use the local_origin frame
	tf::Quaternion orientation = origin2turret.inverse().getRotation();
	robotFrame.setRotation(orientation);

	return robotFrame;
}

tf::Quaternion TransformationUnit::computeTurretRotation(const elikos_ros::RobotRaw& robot){
	//initialization
	tf::Quaternion rotation = tf::createIdentityQuaternion();
	//compute angles
	double roll = -((double) (robot.point.y - cam_height_ / 2) / (double) cam_height_) * cam_fov_v_;
	double pitch = ((double) (robot.point.x - cam_width_ / 2) / (double) cam_width_) * cam_fov_h_;
	//set rotation
	rotation.setRPY(roll , pitch, (double) 0.0);
	return rotation;
}
