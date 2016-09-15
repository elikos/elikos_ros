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
    util::Line line(robot.getPose().getOrigin(), robot.getDirection());
    return segment_.isIntersecting(line);
}

void AbstractLine::evaluate(TargetRobot& robot)
{
    OrientationEvaluation* evaluation = robot.getOrientationEvaluation();
    util::Line line(robot.getPose().getOrigin(), robot.getDirection());
    tf::Point point;
    bool success = segment_.getIntersectionPoint(line, point);
    if (success) {
        evaluation->intersectionPoint_ = point;
        evaluation->lineIntersectionDistance_ = point.distance(robot.getPose().getOrigin());
        evaluation->isGoodIntersection_ = isGoodLineIntersection(robot);
    }
}


}
