//
// Created by olivier on 01/07/16.
//

#include "TargetRobot.h"

#include "Line.h"

#include "AbstractLine.h"
#include <iostream>

namespace ai
{

AbstractLine::AbstractLine()
    : segment_(tf::Point(), tf::Point())
{
}

AbstractLine::AbstractLine(const tf::Point& cornerA, const tf::Point& cornerB)
    : segment_(cornerA, cornerB)
{
}

AbstractLine::~AbstractLine()
{
}

bool AbstractLine::isInThePath(const TargetRobot& robot) const
{
    tf::Pose pose = robot.getPose();

    tf::Quaternion q = pose.getRotation();
    tf::Vector3 orientation = tf::quatRotate(pose.getRotation(), tf::Vector3(1, 0, 0));

    util::Line line(pose.getOrigin(), orientation);
    return segment_.isIntersecting(line);
}

}
