//
// Created by olivier on 01/07/16.
//

#include "TargetRobot.h"

#include "Line.h"
#include "TargetRobot.h"

#include "GreenLine.h"

namespace ai
{

GreenLine::GreenLine(const tf::Point& cornerA, const tf::Point& cornerB)
    : AbstractLine(cornerA, cornerB)
{
}

void GreenLine::concreteEvaluate(TargetRobot& target)
{
    tf::Vector3 direction = target.getDirection();
    tf::Vector3 possibleOrientations[4]  {
        tf::Vector3( direction.x(),  direction.y(), direction.z()),
        tf::Vector3(-direction.x(),  direction.y(), direction.z()),
        tf::Vector3(-direction.x(), -direction.y(), direction.z()),
        tf::Vector3( direction.x(), -direction.y(), direction.z())
    };

    bool directionFound = false;
    int i = 0;
    for (i = 0; i < 4 && !directionFound; ++i)
    {
        util::Line line(target.getPose().getOrigin(), possibleOrientations[i]);
        directionFound = isIntersecting(line);
    }
}

bool GreenLine::isGoodLineIntersection(const TargetRobot& robot)
{
    return true;
}

}