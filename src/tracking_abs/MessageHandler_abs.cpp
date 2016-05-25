#include "MessageHandler_abs.h"

MessageHandler_abs::MessageHandler_abs()
{
	sub1_ = nh_.subscribe("robot_raw_array", 1, &MessageHandler_abs::dispatchMessageRobotRaw, this);
	sub2_ = nh_.subscribe("/mavros/local_position/local", 1, &MessageHandler_abs::dispatchMessagePose, this);
	pubTest_ = nh_.advertise<geometry_msgs::PoseArray>("robot_pose_array",1);
}


MessageHandler_abs::~MessageHandler_abs()
{
}


void MessageHandler_abs::dispatchMessageRobotRaw(const elikos_ros::RobotRawArray::ConstPtr& input)
{
	inputArray_ = *input;
	
	std::cout<<"tracking_abs: robot array size: "<<input->robots.size()<<std::endl;//test
	
	//Les calculs suivant sont temporaires en attendant qu'on les publies.
	geometry_msgs::PoseArray results = transformationUnit_.computeTransformForRobots(inputArray_);
	
	//publish results
	pubTest_.publish(results);
}


void MessageHandler_abs::dispatchMessagePose(const geometry_msgs::PoseStamped::ConstPtr &input)
{
	inputPose_ = *input;
}
