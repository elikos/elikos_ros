//
// Created by olivier on 07/07/16.
//

#include "AggressiveBehavior.h"

namespace ai
{

AggressiveBehavior::~AggressiveBehavior()
{
}

void AggressiveBehavior::generateCommands(CommandQueue& q, Context& context)
{
}

bool AggressiveBehavior::isContextCritical(Context& context)
{
    if (currentTarget_ == nullptr)
    {
        return false;
    }

    TargetOrientationEvaluation evaluation;
    context.getArena()->evaluateTargetOrientation(*currentTarget_, evaluation);
    return evaluation.getGoodIntersection();
}

TargetRobot* AggressiveBehavior::chooseTargetRobot()
{
    return nullptr;
}

void AggressiveBehavior::generateInteractionCommands(CommandQueue& q, Context& context)
{
    TargetOrientationEvaluation evaluation;
    context.getArena()->evaluateTargetOrientation(*currentTarget_, evaluation);


}

}
