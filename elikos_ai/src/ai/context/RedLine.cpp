//
// Created by olivier on 01/07/16.
//

#include "TargetRobot.h"

#include "RedLine.h"

namespace ai
{

RedLine::RedLine(tf::Point& cornerA, tf::Point& cornerB)
        : AbstractLine(cornerA, cornerB)
{
}

RedLine::~RedLine()
{
}

bool RedLine::isGoodLineIntersection(const TargetRobot& target)
{
    // TODO: check target color
    return false;
}

}
