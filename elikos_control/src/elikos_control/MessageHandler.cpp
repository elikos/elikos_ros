#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "MessageHandler.h"
#include "CmdExecutor.h"



MessageHandler::MessageHandler(CmdExecutor& cmdExecutor)
	: cmdExecutor_(cmdExecutor)
{
	sub_ = nh_.subscribe("elikos_trajectory", 1, &MessageHandler::dispatchMessage, this);
}


MessageHandler::~MessageHandler()
{

}

void MessageHandler::dispatchMessage(const elikos_ros::TrajectoryCmd::ConstPtr& input)
{
	lastReceivedCmd_.cmdCode_ = input->cmdCode;
	lastReceivedCmd_.cmdTrajectory_ = input->trajectory;
	lastReceivedCmd_.cmdDestination_ = input->destination;
	lastReceivedCmd_.id_ = lastCmdId_++;
	cmdExecutor_.commandReceived(lastReceivedCmd_);
}
