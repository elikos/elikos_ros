//
// Created by olivier on 01/07/16.
//

#include "AbstractArena.h"

namespace ai
{

const tf::Point AbstractArena::TOP_RIGHT_CORNER{10.0, 10.0, 0.0};
const tf::Point AbstractArena::TOP_LEFT_CORNER{-10.0, 10.0, 0.0};
const tf::Point AbstractArena::BOTTOM_LEFT_CORNER{-10.0, -10.0, 0.0};
const tf::Point AbstractArena::BOTTOM_RIGHT_CORNER{10.0, -10.0, 0.0};


AbstractArena::~AbstractArena()
{
}

TargetRobot* AbstractArena::findHighestPriorityTarget()
{
}

void AbstractArena::resetPriority()
{
    for (int i = 0; i < targets_.size(); ++i)
    {
        targets_[i].setPriority(0.0);
    }

}

TargetRobot* AbstractArena::updateTarget(const elikos_ros::TargetRobot& targetUpdate, int i)
{
    tf::Pose pose;
    tf::poseMsgToTF(targetUpdate.poseOrigin.pose, pose);
    targets_[i].setPose(pose);
    targets_[i].setId(targetUpdate.id);
    targets_[i].setColor(targetUpdate.color);
    return &targets_[i];
}

}
