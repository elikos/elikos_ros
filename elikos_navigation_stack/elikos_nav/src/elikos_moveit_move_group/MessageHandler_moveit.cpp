#include "MessageHandler_moveit.h"

MessageHandler_moveit::MessageHandler_moveit()
{
   sub_ = nh_.subscribe("ai_target", 1, &MessageHandler_moveit::dispatchMessageTarget, this);
   hasTarget_ = false;
}


MessageHandler_moveit::~MessageHandler_moveit()
{
}

void MessageHandler_moveit::dispatchMessageTarget(const geometry_msgs::PoseStamped::ConstPtr &input)
{
	 inputTarget_ = *input;
   hasTarget_ = true;
}

geometry_msgs::PoseStamped MessageHandler_moveit::getTarget()
{
	 return inputTarget_;
}
bool MessageHandler_moveit::getHasTarget()
{
  return hasTarget_;
}
