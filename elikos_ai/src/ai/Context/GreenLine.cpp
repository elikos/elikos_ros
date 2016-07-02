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

    tf::Pose pose = robot.getPose();
    tf::Vector3 orientation = tf::quatRotate(pose.getRotation(), tf::Vector3());
    util::Line line(pose.getOrigin(), orientation);

    tf::Point point;
    bool succes = segment_.getIntersectionPoint(line, point);
    evaluation.setIntersectionPoint(point);
    double distance = point.distance(pose.getOrigin());
    evaluation.setLineIntersectionDistance(distance);
}


}