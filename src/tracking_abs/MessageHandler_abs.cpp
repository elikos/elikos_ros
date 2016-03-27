#include "MessageHandler_abs.h"
#define PI 3.14159265359


MessageHandler_abs::MessageHandler_abs()
{
	sub1_ = nh_.subscribe("robot_raw_array", 1, &MessageHandler_abs::dispatchMessageRobotRaw, this);
	sub2_ = nh_.subscribe("/mavros/local_position/local", 1, &MessageHandler_abs::dispatchMessagePose, this);
}


MessageHandler_abs::~MessageHandler_abs()
{
}


void MessageHandler_abs::dispatchMessageRobotRaw(const elikos_ros::RobotRawArray::ConstPtr& input)
{
	inputArray_ = *input;
	
	//Affichage à la console pour tester
	std::cout<<"tracking_abs: robot array size: "<<input->robots.size()<<std::endl;
	
	double orientationDrone = cv::fastAtan2(inputPose_.pose.orientation.y, inputPose_.pose.orientation.x) / 360 * 2 * PI;
	std::cout<<"tracking_abs: orientation drone: "<<orientationDrone * 360 / (2*PI)<<std::endl;
	//Les calculs suivant sont temporaires en attendant qu'on les publies.
	for(auto robot: inputArray_.robots){
		double orientationRobot = robot.pose.theta;
		if(orientationRobot < orientationDrone)
			orientationRobot += 2 * PI;
		orientationRobot -= orientationDrone;
		//Affichage à la console pour tester
		std::cout<<"tracking_abs: orientation robot: "<<orientationRobot * 360 / (2*PI)<<std::endl;
	}
}


void MessageHandler_abs::dispatchMessagePose(const geometry_msgs::PoseStamped::ConstPtr &input)
{
	inputPose_ = *input;
	//Affichage à la console pour tester
	//std::cout<<"tracking_abs: local_position, orientation.x: "<<input->pose.orientation.x<<std::endl;
}
