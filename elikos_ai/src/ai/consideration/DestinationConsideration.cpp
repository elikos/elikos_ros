//
// Created by olivier on 27/06/16.
//

#include "AbstractArena.h"
#include "DestinationConsideration.h"

namespace ai
{

DestinationConsideration::~DestinationConsideration()
{
}

void DestinationConsideration::evaluatePriority(AbstractArena* arena)
{
    //OrientationEvaluation* evaluation = target.getOrientationEvaluation();
    //target.setPriority((30.0 - evaluation->lineIntersectionDistance_) / 30.0);
}

}
