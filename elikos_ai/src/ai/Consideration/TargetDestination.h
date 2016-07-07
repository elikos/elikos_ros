//
// Created by olivier on 27/06/16.
//

#ifndef AI_RED_LINE_DISTANCE_H
#define AI_RED_LINE_DISTANCE_H

#include "AbstractConsideration.h"
#include "TargetOrientationEvaluation.h"

namespace ai
{

class RedLineDistance : public AbstractConsideration
{
public:
    RedLineDistance() = default;
    RedLineDistance(AbstractArena* arena);
    virtual ~RedLineDistance();
    virtual void evaluatePriority(std::vector<TargetRobot>& targets, const QuadRobot& quad);
private:
    void applyPriorityEvaluation(TargetRobot& robot, const TargetOrientationEvaluation& evaluation);
};

}

#endif //AI_RED_LINE_DISTANCE_H
