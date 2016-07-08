//
// Created by olivier on 27/06/16.
//
#include <cmath>

#include <queue>
#include "TargetDestination.h"
#include "AbstractArena.h"

namespace ai
{

TargetDestination::TargetDestination(AbstractArena* arena)
    : AbstractConsideration(arena)
{
}

TargetDestination::~TargetDestination()
{
}

void TargetDestination::evaluatePriority(std::vector<TargetRobot>& targets, const QuadRobot& quad)
{
    for (int i = 0; i < targets.size(); ++i)
    {
        TargetOrientationEvaluation evaluation;
        arena_->evaluateTargetOrientation(targets[i], evaluation);
        applyPriorityEvaluation(targets[i], evaluation);
    }
}

void TargetDestination::applyPriorityEvaluation(TargetRobot& robot, const TargetOrientationEvaluation& evaluation)
{
    if (!evaluation.getGoodIntersection())
    {
        robot.setPriority((30.0 - evaluation.getLineIntersectionDistance()) / 30.0);
    }
}

}
