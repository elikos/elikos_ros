#include "Robot.h"

#include "FollowClosestTarget.h"


namespace ai
{

FollowClosestTarget::FollowClosestTarget(QuadRobot &quad)
    : Strategy(quad)
{
}

FollowClosestTarget::~FollowClosestTarget()
{
}

Robot* FollowClosestTarget::findTargetSelection()
{
    //Initialize with the first robot.
    tfScalar minDistance = tf::tfDistance(quad_.getPosition(), targets_[0].getPosition());
    int minId = 0;

    //Scan through other robots to find the closest one.
    for (int i = 1; i < targets_.size(); i++)
    {
        tfScalar distance = tf::tfDistance(quad_.getPosition(), targets_[i].getPosition());
        if (distance < minDistance)
        {
            minDistance = distance;
            minId = i;
        }
        resetTarget(i);
    }
    return &targets_[minId];
}

};
