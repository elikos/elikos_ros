//
// Created by olivier on 27/06/16.
//

#include "AbstractArena.h"
#include <iterator>
#include "DestinationConsideration.h"

namespace ai
{

DestinationConsideration::~DestinationConsideration()
{
}

void DestinationConsideration::evaluatePriority(AbstractArena* arena)
{
    std::unordered_map<int, TargetRobot*>::const_iterator it = arena->getTargets().begin();

    //OrientationEvaluation* evaluation = target.getOrientationEvaluation();
    //target.setPriority((30.0 - evaluation->lineIntersectionDistance_) / 30.0);
}

}
