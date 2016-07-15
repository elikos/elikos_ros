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
    q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(quad, currentTarget_)));
    int nLeftRotations = arena_->getNRotationsForOptimalDirection(*currentTarget_);
    switch (nLeftRotations)
    {
        case 1:
            // 180 rotation then right 90 rotation
            q_.push(std::unique_ptr<FrontInteractionCommand>(new FrontInteractionCommand(quad, currentTarget_)));
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

bool AggressiveBehavior::isStateCritical()
{
    currentTarget_ = arena_->findClosestTargetToGoodLine();
    OrientationEvaluation* evaluation = currentTarget_->getOrientationEvaluation();
    bool isCritical = !evaluation->isGoodIntersection_;
    if (!isCritical) {
        q_.clear();
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena_->getQuad(), currentTarget_)));
    }
    return isCritical;
}


}
