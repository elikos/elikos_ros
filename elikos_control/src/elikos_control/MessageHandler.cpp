#include "MessageHandler.h"

MessageHandler::MessageHandler()
{
	sub_ = nh_.subscribe("elikos_trajectory", 1, &MessageHandler::dispatchMessage, this);
}


MessageHandler::~MessageHandler()
{

}

void MessageHandler::dispatchMessage(const elikos_ros::TrajectoryCmd::ConstPtr& input)
{

}
