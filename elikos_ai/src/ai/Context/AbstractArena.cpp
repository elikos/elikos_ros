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

TargetRobot *AbstractArena::findHighestPriorityTarget()
{

}

void AbstractArena::resetPriority()
{
    for (int i = 0; i < targets_.size(); ++i)
    {
        targets_[i].setPriority(0.0);
    }

}

void AbstractArena::updateTargets(const elikos_ros::TargetRobotArray::ConstPtr &input)
{

}

}
