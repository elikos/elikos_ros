//
// Created by olivier on 07/07/16.
//

#include "AbstractArena.h"
#include "CommandTypes.h"

#include "PreventiveBehavior.h"

namespace ai
{
PreventiveBehavior::PreventiveBehavior(AbstractArena* arena)
        : AbstractBehavior(arena)
{
}

PreventiveBehavior::~PreventiveBehavior()
{
}

void PreventiveBehavior::generateCommands()
{
    TargetRobot* target = arena_->findHighestPriorityTarget();
    QuadRobot* quad = &arena_->getQuad();
    q_.clear();
    q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(quad, target)));
    q_.push(std::unique_ptr<TopInteractionCommand>(new TopInteractionCommand(quad, target)));
    q_.push(std::unique_ptr<ObservationCommand>(new ObservationCommand(quad, target)));
}

bool PreventiveBehavior::isStateCritical()
{
    TargetRobot* target = arena_->findHighestPriorityTarget();
    return target->getPriority() > MAX_ACCEPTABLE_PRIORITY;
}

}
