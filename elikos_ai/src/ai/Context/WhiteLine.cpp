//
// Created by olivier on 01/07/16.
//

#include "TargetRobot.h"
#include "Line.h"

#include "WhiteLine.h"

namespace ai
{

WhiteLine::WhiteLine(const tf::Point& cornerA, const tf::Point& cornerB)
        : AbstractLine(cornerA, cornerB)
{
}

WhiteLine::~WhiteLine()
{
}

void WhiteLine::evaluate(const TargetRobot& robot, TargetOrientationEvaluation& evaluation)
{
    util::Line line(robot.getPose().getOrigin(), robot.getOrientation());

    tf::Point point;
    bool success = segment_.getIntersectionPoint(line, point);
    assert(success);

    evaluation.setIntersectionPoint(point);
    double distance = point.distance(robot.getPose().getOrigin());
    evaluation.setLineIntersectionDistance(distance);
    evaluation.setGoodIntersection(false);
}

}
