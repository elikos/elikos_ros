#include "TransformationUnit.h"

TransformationUnit::TransformationUnit(){
	pub_ = nh_.advertise<elikos_ros::TargetRobotArray>("target_robot_array",1);
}

geometry_msgs::PoseArray TransformationUnit::computeTransformForRobots(elikos_ros::RobotRawArray robotArray){
    elikos_ros::TargetRobotArray targetArray;
    
    //results for simulation in rviz
    geometry_msgs::PoseArray results;
    //Stamp the array
    results.header.stamp = ros::Time();
    results.header.frame_id = "absolute_origin";
	for(auto robot: robotArray.robots){
		//Define the camera2turret turret transform
		tf::Transform camera2turret = tf::Transform::getIdentity();
		//Same origin than the camera
		camera2turret.setOrigin(tf::Vector3(0, 0, 0));
		//Rotation in function of the detection with computer vision
		tf::Quaternion rotation = computeTurretRotation(robot);
		camera2turret.setRotation(rotation);
		
		
		//Get the origin to camera transform
		tf::StampedTransform origin2camera;
		try {
			tf_listener_.lookupTransform("absolute_origin", "camera", ros::Time(0), origin2camera);
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("TransformationUnit::computeTransformForRobots() exception : %s",ex.what());
			ros::Duration(1.0).sleep();
		}
		//Get the fcu to camera transform
        tf::StampedTransform fcu2camera;
		try {
			tf_listener_.lookupTransform("fcu", "camera", ros::Time(0), fcu2camera);
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("TransformationUnit::computeTransformForRobots() exception : %s",ex.what());
			ros::Duration(1.0).sleep();
		}
		
		// Compute the origin to turret transform
		tf::Transform origin2turret = origin2camera * camera2turret;
		
		// Find the robot2turret transform
		tf::Transform turret2robot = computeRobotTransform(origin2turret);
		
		//Compute the robot poses
		tf::Transform origin2robot = origin2camera * camera2turret * turret2robot;
		tf::Transform fcu2robot = fcu2camera * camera2turret * turret2robot ;
		
		//Compute direction
		tf::Transform direction_transform = tf::Transform::getIdentity();
		tf::Quaternion robot_theta = tf::createIdentityQuaternion();
		double direction = 0; // if the direction is 0. the robot has no known direction.
		for(auto old : oldArray_.targets){
			//if the id is the same and the difference of the position is high enough (arbitrary criterion : tolerance).
			double tolerance = 0.02; //in meters
			if(old.id == robot.id && (sqrt(pow(origin2robot.getOrigin().getY() - old.poseOrigin.pose.position.y,2) + pow(origin2robot.getOrigin().getX() - old.poseOrigin.pose.position.x,2))>tolerance)){
				direction = cv::fastAtan2(origin2robot.getOrigin().getY() - old.poseOrigin.pose.position.y, origin2robot.getOrigin().getX() - old.poseOrigin.pose.position.x) / 360 * 2 *PI;
			}
		}
		robot_theta.setRPY(0,0,direction);
		direction_transform.setRotation(robot_theta);
		//Apply absolute_origin and direction transform.
		origin2robot = origin2robot * direction_transform;
		fcu2robot = fcu2robot * direction_transform;
		
		//Define the pose and emplace it in the collection
		geometry_msgs::PoseStamped robotPoseOrigin;
		robotPoseOrigin.header.stamp = ros::Time();
		robotPoseOrigin.header.frame_id = "absolute_origin";
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
	oldArray_ = targetArray;
    return results;
}
tf::Transform TransformationUnit::computeRobotTransform(tf::Transform origin2turret){	
	// Get the smallest angle between the turret and the z axis
	// - First get the vector pointing towards the z axis of the turret
	tf::Vector3 vect_z = tf::quatRotate(origin2turret.getRotation(), tf::Vector3(0, 0, 1));
	// - Then find it's angle with the camera's resting position (-z)
	tf::Vector3 zAxis(0, 0, -1);
	double zAxis_turret_angle = zAxis.angle(vect_z);
	
	// Initiallization on the robot frame coordinate
	tf::Transform robotFrame = tf::Transform::getIdentity();
	robotFrame.setOrigin(tf::Vector3(0, 0, 0));
	
	// Get distance from turret to target (using angle and altitude) Should we use a mavros topic?
	double altitude = origin2turret.getOrigin().getZ();
	double distance_from_robot = altitude / cos(zAxis_turret_angle);
	robotFrame.setOrigin(tf::Vector3(0, 0, distance_from_robot));
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
	double roll = -((double) (robot.point.y - CAM_HEIGHT / 2) / (double) CAM_HEIGHT) * CAMERA_FOV_V;
	double pitch = ((double) (robot.point.x - CAM_WIDTH / 2) / (double) CAM_WIDTH) * CAMERA_FOV_H;
	//set rotation
	rotation.setRPY(roll , pitch, (double) 0.0);
	return rotation;
}

