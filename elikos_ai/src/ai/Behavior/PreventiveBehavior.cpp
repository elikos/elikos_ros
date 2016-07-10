//
// Created by olivier on 07/07/16.
//

#include "Agent.h"
#include "CommandTypes.h"

#include "PreventiveBehavior.h"


namespace ai
{

PreventiveBehavior::~PreventiveBehavior()
{
}

void PreventiveBehavior::generateCommands(CommandQueue& command, Context& context)
{
    TargetRobot* target = context.findHighestPriorityTarget();
    QuadRobot* quad = &context.getQuad();
    command.push(std::unique_ptr<ObservationCommand>(new ObservationCommand(quad, target)));
    command.push(std::unique_ptr<MovementCommand>(new MovementCommand(quad, target)));
    command.push(std::unique_ptr<TopInteractionCommand>(new ObservationCommand(quad, target)));
}

bool PreventiveBehavior::isContextCritical(Context& context)
{
    bool isAcceptable = true;
    for (int i = 0; i < context.getTargets().size() && isAcceptable; ++i)
    {
        TargetOrientationEvaluation* evaluation = context.getTargets()[i].getOrientationEvaluation();
        isAcceptable = evaluation->getLineIntersectionDistance() > MIN_ACCEPTABLE_LINE_DISTANCE;
    }
    return isAcceptable;
}

}
