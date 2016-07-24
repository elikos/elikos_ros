//
// Created by olivier on 07/07/16.
//

#include "AbstractArena.h"
#include "CommandTypes.h"

#include "AggressiveBehavior.h"

namespace ai
{

AggressiveBehavior::AggressiveBehavior(AbstractArena* arena)
    : AbstractBehavior(arena)
{
}


AggressiveBehavior::~AggressiveBehavior()
{
}

void AggressiveBehavior::generateCommands()
{
    currentTarget_ = arena_->findClosestTargetToGoodLine();
    QuadRobot* quad = &arena_->getQuad();
    q_.clear();

    if (currentTarget_ == nullptr) {
        searchForTargets();
        return;
    }

    q_.push(std::unique_ptr<FollowCommand>(new FollowCommand(quad, currentTarget_)));
    int nLeftRotations = arena_->getNRotationsForOptimalDirection(*currentTarget_);
    switch (nLeftRotations)
    {
        case 1:
            // 180 rotation then right 90 rotation
            q_.push(std::unique_ptr<FrontInteractionCommand>(new FrontInteractionCommand(quad, currentTarget_)));
            q_.push(std::unique_ptr<FollowCommand>(new FollowCommand(quad, currentTarget_)));
            q_.push(std::unique_ptr<TopInteractionCommand>(new TopInteractionCommand(quad, currentTarget_)));
            break;

        case 2:
            // 180 rotation
            q_.push(std::unique_ptr<FrontInteractionCommand>(new FrontInteractionCommand(quad, currentTarget_)));
            break;
        case 3:
            // single 90 rotation
            q_.push(std::unique_ptr<TopInteractionCommand>(new TopInteractionCommand(quad, currentTarget_)));
            break;
        default:
            // do nothing
            break;
    }
}

int AggressiveBehavior::resolveCurrentStateLevel()
{
    currentTarget_ = arena_->findClosestTargetToGoodLine();

    if (currentTarget_ == nullptr) {
        return 0;
    }

    OrientationEvaluation* evaluation = currentTarget_->getOrientationEvaluation();
    int stateLevel = 0;
    if (evaluation->isGoodIntersection_) {
        // TODO: Something better should be done here
        // We need to clear the q if the target is going in the good direction or the ai will keep executing the
        // interaction commands that are still there.
        q_.clear();
        q_.push(std::unique_ptr<FollowCommand>(new FollowCommand(&arena_->getQuad(), currentTarget_)));
    } else {
        stateLevel = 1;
    }
    return stateLevel;
}

}
