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
    evaluation.setGoodIntersection(false);

    tf::Pose pose = robot.getPose();
    tf::Vector3 orientation = tf::quatRotate(pose.getRotation(), tf::Vector3(1, 0, 0));
    util::Line line(pose.getOrigin(), orientation);

    tf::Point point;
    bool success = segment_.getIntersectionPoint(line, point);
    assert(success);
    evaluation.setIntersectionPoint(point);
    double distance = point.distance(pose.getOrigin());
    evaluation.setLineIntersectionDistance(distance);
}

}
