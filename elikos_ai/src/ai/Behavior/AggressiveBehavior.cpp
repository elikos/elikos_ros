//
// Created by olivier on 07/07/16.
//

#include "AggressiveBehavior.h"
#include "CommandTypes.h"

namespace ai
{

AggressiveBehavior::AggressiveBehavior()
{
    q_.push(std::unique_ptr<ObservationCommand>(new ObservationCommand(quad, currentTarget_)));
}

AggressiveBehavior::~AggressiveBehavior()
{
}

void AggressiveBehavior::generateCommands(Context& context)
{
    QuadRobot* quad = &context.getQuad();
    q_.clear();
    q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(quad, currentTarget_)));
    int nLeftRotations = context.getArena()->getNRotationsForOptimalDirection(*currentTarget_);
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

bool AggressiveBehavior::isContextCritical(Context& context)
{
    if (currentTarget_ == nullptr)
    {
        return false;
    }
    TargetOrientationEvaluation evaluation;
    return evaluation.getGoodIntersection();
}

TargetRobot* AggressiveBehavior::chooseTargetRobot()
{
    return nullptr;
}

void AggressiveBehavior::generateInteractionCommands(Context& context)
{
    q_.clear();
}



}
