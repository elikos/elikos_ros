#include "MessageHandler.h"

MessageHandler::MessageHandler()
{
    std::string cam_name = ros::this_node::getName();
    cam_name = cam_name.substr(0, cam_name.size()-std::string("_transformation").size());
	sub_ = nh_.subscribe("/"+cam_name+"/elikos_robot_raw_array", 1, &MessageHandler::dispatchMessageRobotRaw, this);

	pubTest_ = nh_.advertise<geometry_msgs::PoseArray>("elikos_robot_pose_array",1);
	pub_ = nh_.advertise<elikos_ros::TargetRobotArray>("elikos_target_robot_array",1);
}


MessageHandler::~MessageHandler()
{
}


void MessageHandler::dispatchMessageRobotRaw(const elikos_ros::RobotRawArray::ConstPtr& input)
{
	// TODO transformations.

	//Le vecteur results est pour l'affichage dans rviz.
	//geometry_msgs::PoseArray results = transformationUnit_.computeTransformForRobots(*input);

	//publish results
	//pubTest_.publish(results);
}
