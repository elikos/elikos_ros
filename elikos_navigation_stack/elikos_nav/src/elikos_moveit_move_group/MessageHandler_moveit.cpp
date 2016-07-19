#include "MessageHandler_moveit.h"

MessageHandler_moveit::MessageHandler_moveit()
{
   sub_ = nh_.subscribe("elikos_ai_cmd", 1, &MessageHandler_moveit::dispatchMessageTarget, this);
   hasTarget_ = false;
}

MessageHandler_moveit::~MessageHandler_moveit()
{
}

void MessageHandler_moveit::dispatchMessageTarget(const geometry_msgs::PoseStamped::ConstPtr &input)
{
  if(!hasTarget_ || (inputTarget_.pose.position.x!=(*input).pose.position.x ||
      inputTarget_.pose.position.y!=(*input).pose.position.y ||
      inputTarget_.pose.position.z!=(*input).pose.position.z))
  {
     move_group_.move(*input);
	   inputTarget_ = *input;
     hasTarget_ = true;
  }
}
