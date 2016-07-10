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

bool isGoodLineIntersection(const TargetRobot& robot)
{
    return true;
}

}