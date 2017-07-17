//
// Created by olivier on 07/07/16.
//

#include "AbstractArena.h"
#include "CommandTypes.h"
#include "FollowCommand.h"
#include "TopInteractionCommand.h"
#include "Configuration.h"

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
        //q_.push(std::unique_ptr<TakeOffCommand>(new TakeOffCommand(&arena_->getQuad(), &tf_listener_)));
        q_.push(std::unique_ptr<ObservationCommand>(new ObservationCommand(quad, target)));
    }
}

int PreventiveBehavior::resolveCurrentStateLevelConcrete()
{
    int stateLevel = 1;
    TargetRobot* target = arena_->findHighestPriorityTarget();
    if (target != nullptr)
    {
        double priority = target->getPriority();
        if (priority > MIN_ACCEPTABLE_PRIORITY)
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
