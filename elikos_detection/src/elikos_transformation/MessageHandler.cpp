#include "MessageHandler.h"

MessageHandler::MessageHandler()
{
	sub_ = nh_.subscribe("elikos_robot_raw_array", 1, &MessageHandler::dispatchMessageRobotRaw, this);

	pubTest_ = nh_.advertise<geometry_msgs::PoseArray>("elikos_robot_pose_array",1);
}


MessageHandler::~MessageHandler()
{
}


void MessageHandler::dispatchMessageRobotRaw(const elikos_ros::RobotRawArray::ConstPtr& input)
{

	newArray_ = *input;

	//Le vecteur results est pour l'affichage dans rviz.
	geometry_msgs::PoseArray results = transformationUnit_.computeTransformForRobots(newArray_);

	//publish results
	pubTest_.publish(results);
}
