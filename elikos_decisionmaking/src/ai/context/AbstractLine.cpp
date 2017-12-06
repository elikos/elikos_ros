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
    : util::Segment(tf::Point(), tf::Point())
{
}

AbstractLine::AbstractLine(tf::Point& cornerA, tf::Point& cornerB)
    : util::Segment(cornerA, cornerB)
{
}

AbstractLine::~AbstractLine()
{
}

bool AbstractLine::isInThePath(const TargetRobot& robot) const
{
    util::Line line(robot.getPose().getOrigin(), robot.getDirection());
    return isIntersecting(line);
}

void AbstractLine::evaluate(TargetRobot& robot)
{
    OrientationEvaluation* evaluation = robot.getOrientationEvaluation();
    util::Line line(robot.getPose().getOrigin(), robot.getDirection());
    tf::Point point;
    bool success = getIntersectionPoint(line, point);
    if (success) {
        evaluation->intersectionPoint_ = point;
        evaluation->lineIntersectionDistance_ = point.distance(robot.getPose().getOrigin());
        evaluation->isGoodIntersection_ = isGoodLineIntersection(robot);
    }
}


}
