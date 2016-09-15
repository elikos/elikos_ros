//
// Created by olivier on 07/07/16.
//

#include "AbstractArena.h"
#include "CommandTypes.h"
#include "FollowCommand.h"

#include "PreventiveBehavior.h"

namespace ai
{
PreventiveBehavior::PreventiveBehavior(bool isEnabled)
    : AbstractBehavior(isEnabled)
{
}

PreventiveBehavior::~PreventiveBehavior()
{
}

void PreventiveBehavior::generateCommands(AbstractArena* arena)
{
    q_.clear();
    TargetRobot* target = arena->findHighestPriorityTarget();
    QuadRobot* quad = &arena->getQuad();

    if (target != nullptr) {
        q_.push(std::unique_ptr<FollowCommand>(new FollowCommand(quad, target)));
        q_.push(std::unique_ptr<TopInteractionCommand>(new TopInteractionCommand(quad, target)));
        q_.push(std::unique_ptr<ObservationCommand>(new ObservationCommand(quad, target)));
    }
}

int PreventiveBehavior::resolveCurrentStateLevelConcrete(AbstractArena* arena)
{
    int stateLevel = 1;
    TargetRobot* target = arena->findHighestPriorityTarget();
    if (target != nullptr)
    {
        double priority = target->getPriority();
        if (priority > MAX_ACCEPTABLE_PRIORITY)
        {
            stateLevel = 3;
        }
        else if (priority > 0.0)
        {
            stateLevel = 2;
        }
    }
    return stateLevel;
}

}
