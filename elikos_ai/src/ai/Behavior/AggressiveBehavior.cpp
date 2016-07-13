//
// Created by olivier on 07/07/16.
//

#include "AbstractArena.h"

#include "AggressiveBehavior.h"
#include "CommandTypes.h"

namespace ai
{

AggressiveBehavior::~AggressiveBehavior()
{
}

void AggressiveBehavior::generateCommands(AbstractArena* arena)
{
    QuadRobot* quad = &arena->getQuad();
    q_.clear();
    q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(quad, currentTarget_)));
    int nLeftRotations = arena->getNRotationsForOptimalDirection(*currentTarget_);
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

bool AggressiveBehavior::isStateCritical(AbstractArena* arena)
{
    if (currentTarget_ == nullptr)
    {
        return false;
    }
    OrientationEvaluation* evaluation = currentTarget_->getOrientationEvaluation();
    return evaluation->isGoodIntersection_;
}

TargetRobot* AggressiveBehavior::chooseTargetRobot()
{
    return nullptr;
}

void AggressiveBehavior::generateInteractionCommands(AbstractArena* arena)
{
    q_.clear();
}



}
