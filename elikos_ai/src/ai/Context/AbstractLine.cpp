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
    util::Line line(robot.getPose().getOrigin(), robot.getOrientation());
    return segment_.isIntersecting(line);
}

void AbstractLine::evaluate(TargetRobot& robot)
{
    TargetOrientationEvaluation* evaluation = robot.getOrientationEvaluation();
    util::Line line(robot.getPose().getOrigin(), robot.getOrientation());
    tf::Point point;

    bool success = segment_.getIntersectionPoint(line, point);
    if (success) {
        evaluation->setIntersectionPoint(point);
        double distance = point.distance(robot.getPose().getOrigin());

        evaluation->setLineIntersectionDistance(distance);
        evaluation->setGoodIntersection(false);
        evaluation->setGoodIntersection(isGoodLineIntersection(robot));
    }
    concreteEvaluate(robot);
}

void AbstractLine::concreteEvaluate(TargetRobot& robot)
{
}

}
