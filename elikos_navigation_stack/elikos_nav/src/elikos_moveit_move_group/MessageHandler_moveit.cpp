#include "MessageHandler_moveit.h"

MessageHandler_moveit::MessageHandler_moveit()
{
   sub_ = nh_.subscribe("elikos_robot_pose_array", 1, &MessageHandler_moveit::dispatchMessageTarget, this);
   hasTarget_ = false;
}

MessageHandler_moveit::~MessageHandler_moveit()
{
}

void MessageHandler_moveit::dispatchMessageTarget(const geometry_msgs::PoseArray::ConstPtr &input)
{
   geometry_msgs::PoseStamped pose;
   pose.header = (*input).header;
   pose.pose = (*input).poses[0];
	 inputTarget_ = pose;
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
