//
// Created by olivier on 27/06/16.
//
#include <cmath>

#include <queue>
#include "TargetDestination.h"
#include "AbstractArena.h"

namespace ai
{

TargetDestination::~TargetDestination()
{
}

void TargetDestination::evaluatePriority(TargetRobot& target)
{
    TargetOrientationEvaluation* evaluation = target.getOrientationEvaluation();
    target.setPriority((30.0 - evaluation->getLineIntersectionDistance()) / 30.0);
}

}
