//
// Created by olivier on 01/07/16.
//

#include "Line.h"
#include "TargetRobot.h"

#include "GreenLine.h"

namespace ai
{

GreenLine::GreenLine(const tf::Point& cornerA, const tf::Point& cornerB)
    : AbstractLine(cornerA, cornerB)
{
}

void GreenLine::evaluate(const TargetRobot& robot, TargetOrientationEvaluation& evaluation)
{
    evaluation.setGoodIntersection(true);
    util::Line line(robot.getPose().getOrigin(), robot.getOrientation());
    tf::Point point;
    bool success = segment_.getIntersectionPoint(line, point);
    assert(success);
    evaluation.setIntersectionPoint(point);
    double distance = point.distance(robot.getPose().getOrigin());
    evaluation.setLineIntersectionDistance(distance);
}


}