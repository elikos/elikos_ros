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

bool WhiteLine::isGoodLineIntersection(const TargetRobot& robot)
{
    return false;
}

}
