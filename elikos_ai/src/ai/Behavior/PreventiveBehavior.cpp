//
// Created by olivier on 07/07/16.
//

#include "AbstractArena.h"
#include "CommandTypes.h"
#include "FollowCommand.h"

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
    q_.clear();
    TargetRobot* target = arena_->findHighestPriorityTarget();
    QuadRobot* quad = &arena_->getQuad();

    if (target != nullptr) {
        q_.push(std::unique_ptr<FollowCommand>(new FollowCommand(quad, target)));
        q_.push(std::unique_ptr<TopInteractionCommand>(new TopInteractionCommand(quad, target)));
        q_.push(std::unique_ptr<ObservationCommand>(new ObservationCommand(quad, target)));
    }
}

int PreventiveBehavior::resolveCurrentStateLevel()
{
    TargetRobot* target = arena_->findHighestPriorityTarget();
    int stateLevel = 0;
    if (target != nullptr) {
        double priority = target->getPriority();
        if (priority > MAX_ACCEPTABLE_PRIORITY) {
            stateLevel = 2;
        } else if (priority > 0.0) {
            stateLevel = 1;
        }
    }
    return stateLevel;
}

}
