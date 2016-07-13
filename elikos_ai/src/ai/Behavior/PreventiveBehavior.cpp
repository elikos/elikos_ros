//
// Created by olivier on 07/07/16.
//

#include "AbstractArena.h"
#include "CommandTypes.h"

#include "PreventiveBehavior.h"

namespace ai
{

PreventiveBehavior::~PreventiveBehavior()
{
}

void PreventiveBehavior::generateCommands(AbstractArena* arena)
{
    TargetRobot* target = arena->findHighestPriorityTarget();
    QuadRobot* quad = &arena->getQuad();
    q_.clear();
    q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(quad, target)));
    q_.push(std::unique_ptr<TopInteractionCommand>(new TopInteractionCommand(quad, target)));
    q_.push(std::unique_ptr<ObservationCommand>(new ObservationCommand(quad, target)));
}

bool PreventiveBehavior::isStateCritical(AbstractArena* arena)
{
    bool isAcceptable = true;
    std::vector<TargetRobot>& targets = arena->getTargets();
    for (int i = 0; i < targets.size() && isAcceptable; ++i)
    {
        OrientationEvaluation* evaluation = targets[i].getOrientationEvaluation();
        isAcceptable = evaluation->lineIntersectionDistance_ > MIN_ACCEPTABLE_LINE_DISTANCE;
    }
    return isAcceptable;
}

}
