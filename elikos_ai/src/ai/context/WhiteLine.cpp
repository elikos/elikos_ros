//
// Created by olivier on 01/07/16.
//

#include "TargetRobot.h"
#include "Line.h"

#include "WhiteLine.h"

namespace ai
{

WhiteLine::WhiteLine(tf::Point& cornerA, tf::Point& cornerB)
        : AbstractLine(cornerA, cornerB)
{
}

WhiteLine::~WhiteLine()
{
}

bool WhiteLine::isGoodLineIntersection(const TargetRobot& target)
{
    // TODO: check target color
    return false;
}

}
