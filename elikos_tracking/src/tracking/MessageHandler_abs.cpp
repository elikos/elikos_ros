#include "MessageHandler_abs.h"

MessageHandler_abs::MessageHandler_abs()
{
	sub_ = nh_.subscribe("elikos_robot_raw_array", 1, &MessageHandler_abs::dispatchMessageRobotRaw, this);
	//pubTest_ = nh_.advertise<geometry_msgs::PoseArray>("robot_pose_array",1);
}


MessageHandler_abs::~MessageHandler_abs()
{
}


void MessageHandler_abs::dispatchMessageRobotRaw(const elikos_ros::RobotRawArray::ConstPtr& input)
{

	newArray_ = *input;

	//Le vecteur results est pour l'affichage dans rviz.
	geometry_msgs::PoseArray results = transformationUnit_.computeTransformForRobots(newArray_);

	//publish results
	//pubTest_.publish(results);
}
