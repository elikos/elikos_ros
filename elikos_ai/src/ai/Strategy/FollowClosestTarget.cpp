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
    tfScalar minDistance = quad_.getDistance(&targets_[0]);
    int minId = 0;

    //Scan through other robots to find the closest one.
    for (int i = 1; i < targets_.size(); ++i)
    {
        tfScalar distance = quad_.getDistance(&targets_[i]);
        if (distance < minDistance)
        {
            minDistance = distance;
            minId = i;
        }
    }
    return &targets_[minId];
}

};
