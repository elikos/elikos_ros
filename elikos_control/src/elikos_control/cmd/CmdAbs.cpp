#include "CmdAbs.h"

CmdAbs::CmdAbs(ros::NodeHandle* nh, int id)
    : nh_(nh), id_(id)
{
    isAborted_ = false;
    statePubCommand_ = nh_->advertise<std_msgs::String>("/elikos_control_state_command", 1);
}

void CmdAbs::ajustement(geometry_msgs::Pose destination, trajectory_msgs::MultiDOFJointTrajectory trajectory)
{
    cmdDestination_ = destination;
    cmdTrajectory_ = trajectory;
}