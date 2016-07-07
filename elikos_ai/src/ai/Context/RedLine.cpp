//
// Created by olivier on 01/07/16.
//

#include "RedLine.h"

namespace ai
{

RedLine::RedLine(const tf::Point& cornerA, const tf::Point& cornerB)
        : AbstractLine(cornerA, cornerB)
{
}

RedLine::~RedLine()
{
}

void RedLine::evaluate(const TargetRobot& robot, TargetOrientationEvaluation& evaluation)
{

}


}
