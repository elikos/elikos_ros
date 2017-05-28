#include "CmdAbs.h"

CmdAbs::CmdAbs(ros::NodeHandle* nh, int id)
    : nh_(nh), id_(id)
{
    isAborted_ = false;
}

void CmdAbs::ajustement(geometry_msgs::Pose destination, trajectory_msgs::MultiDOFJointTrajectory trajectory)
{
    ///
}