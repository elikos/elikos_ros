//
// Created by olivier on 27/06/16.
//

#include "AbstractArena.h"
#include "TargetDestination.h"

namespace ai
{

TargetDestination::~TargetDestination()
{
}

void TargetDestination::evaluatePriority(TargetRobot& target, AbstractArena* arena)
{
    OrientationEvaluation* evaluation = target.getOrientationEvaluation();
    target.setPriority((30.0 - evaluation->lineIntersectionDistance_) / 30.0);
}

}
