#include "TransformationUnit.h"

TransformationUnit::TransformationUnit(){
	pub_ = nh_.advertise<elikos_ros::TargetRobotArray>("target_robot_array",1);
	// Set up the camera's position relative to the fcu
	//camera_.setOrigin(tf::Vector3(-0.10, 0, -0.05));
	//camera_.setRotation(tf::Quaternion(tf::Vector3(0, 1, 0), PI / 2));//camera under the pix4
	camera_.setOrigin(tf::Vector3(0.10, 0, 0));
	camera_.setRotation(tf::Quaternion(-PI/2,-PI/2,0));//in front of the pix4
}

geometry_msgs::PoseArray TransformationUnit::computeTransformForRobots(elikos_ros::RobotRawArray robotArray){
    elikos_ros::TargetRobotArray targetArray;
    
    //results for simulation in rviz
    geometry_msgs::PoseArray results;
    //Stamp the array
    results.header.stamp = ros::Time();
    results.header.frame_id = "local_origin";
	for(auto robot: robotArray.robots){
		//Define the turret2camera turret transform
		tf::Transform turret = tf::Transform::getIdentity();
		//Same origin than the camera
		turret.setOrigin(tf::Vector3(0, 0, 0));
		//Rotation in function of the detection with computer vision
		tf::Quaternion rotation = computeTurretRotation(robot);
		turret.setRotation(rotation);
		
		
		//Get the origin to fcu transform
		tf::StampedTransform fcu;
		try {
			tf_listener_.lookupTransform("local_origin", "fcu", ros::Time(0), fcu);
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("TransformationUnit::computeTransformForRobots() exception : %s",ex.what());
			ros::Duration(1.0).sleep();
		}
		
		// Compute the origin to turret transform
		tf::Transform origin2turret = fcu * camera_ * turret;
		
		// Find the robot2turret transform
		tf::Transform robot2turret = computeRobotTransform(origin2turret);
		//Set the rotation extracted form the image
		tf::Transform theta_transform = tf::Transform::getIdentity();
		tf::Quaternion robot_theta = tf::createIdentityQuaternion();
		robot_theta.setRPY(0,0,robot.pose.theta);
		theta_transform.setRotation(robot_theta);
		
		//Compute the robot poses
		tf::Transform origin2robot = fcu * camera_ * turret * robot2turret * theta_transform;
		tf::Transform fcu2robot = camera_ * turret * robot2turret * theta_transform;
		
		//Define the pose and emplace it in the collection
		geometry_msgs::PoseStamped robotPoseOrigin;
		robotPoseOrigin.header.stamp = ros::Time();
		robotPoseOrigin.header.frame_id = "local_origin";
		robotPoseOrigin.pose.position.x = origin2robot.getOrigin().getX();
		robotPoseOrigin.pose.position.y = origin2robot.getOrigin().getY();
		robotPoseOrigin.pose.position.z = origin2robot.getOrigin().getZ();
		tf::quaternionTFToMsg(origin2robot.getRotation(), robotPoseOrigin.pose.orientation);
		results.poses.push_back(robotPoseOrigin.pose);
		
		geometry_msgs::PoseStamped robotPoseFcu;
		robotPoseOrigin.header.stamp = ros::Time();
		robotPoseOrigin.header.frame_id = "fcu";
		robotPoseFcu.pose.position.x = fcu2robot.getOrigin().getX();
		robotPoseFcu.pose.position.y = fcu2robot.getOrigin().getY();
		robotPoseFcu.pose.position.z = fcu2robot.getOrigin().getZ();
		tf::quaternionTFToMsg(fcu2robot.getRotation(), robotPoseFcu.pose.orientation);
		
		elikos_ros::TargetRobot target;
		target.id = robot.id;
		target.color = robot.color;
		target.poseOrigin = robotPoseOrigin;
		target.poseFcu = robotPoseFcu;
		targetArray.targets.push_back(target);
	}
	pub_.publish(targetArray);
    return results;
}
tf::Transform TransformationUnit::computeRobotTransform(tf::Transform origin2turret){	
	// Get the smallest angle between the turret and the z axis
	// - First get the vector pointing towards the x axis of the turret
	tf::Vector3 vect_x = tf::quatRotate(origin2turret.getRotation(), tf::Vector3(1, 0, 0));
	// - Then find it's angle with the camera's resting position (x pointing straight down (-z))
	tf::Vector3 zAxis(0, 0, -1);
	double zAxis_turret_angle = zAxis.angle(vect_x);
	
	// Initiallization on the robot frame coordinate
	tf::Transform robotFrame = tf::Transform::getIdentity();
	robotFrame.setOrigin(tf::Vector3(0, 0, 0));
	
	// Get distance from turret to target (using angle and altitude) Should we use a mavros topic?
	double altitude = origin2turret.getOrigin().getZ();
	double distance_from_robot = altitude / cos(zAxis_turret_angle);
	robotFrame.setOrigin(tf::Vector3(distance_from_robot, 0, 0));
	//Set the orientation
	//the robots are on the ground, so we use the local_origin frame
	tf::Quaternion orientation = origin2turret.inverse().getRotation();
	robotFrame.setRotation(orientation);
	
	return robotFrame;
}

tf::Quaternion TransformationUnit::computeTurretRotation(elikos_ros::RobotRaw robot){
	//initialization
	tf::Quaternion rotation = tf::createIdentityQuaternion();
	//compute angles
	double pitch = ((double) (robot.pose.y - CAM_HEIGHT / 2) / (double) CAM_HEIGHT) * CAMERA_FOV_V;
	double yaw = -((double) (robot.pose.x - CAM_WIDTH / 2) / (double) CAM_WIDTH) * CAMERA_FOV_H;
	//set rotation
	rotation.setRPY((double) 0.0, pitch, yaw);
	return rotation;
}

