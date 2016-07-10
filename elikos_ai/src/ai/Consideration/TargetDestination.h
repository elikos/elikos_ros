//
// Created by olivier on 27/06/16.
//

#ifndef AI_RED_LINE_DISTANCE_H
#define AI_RED_LINE_DISTANCE_H

#include "AbstractConsideration.h"
#include "TargetOrientationEvaluation.h"

namespace ai
{

class AbstractArena;

class TargetDestination : public AbstractConsideration
{
public:
    TargetDestination() = default;
    virtual ~TargetDestination();
    virtual void evaluatePriority(TargetRobot& target);
};

}

#endif //AI_RED_LINE_DISTANCE_H
