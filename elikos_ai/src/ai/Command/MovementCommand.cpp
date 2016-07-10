//
// Created by olivier on 28/06/16.
//

#include "MessageHandler.h"

#include "MovementCommand.h"
#include "CommandQueue.h"

namespace ai
{

MovementCommand::MovementCommand(QuadRobot* quad, TargetRobot* target)
    : AbstractCommand(quad, target)
{
}

MovementCommand::~MovementCommand()
{
}

bool MovementCommand::execute()
{
    tf::Vector3 destination = target_->getPose().getOrigin();
    destination.setZ(FLIGHT_HEIGHT);
    MessageHandler::getInstance()->sendDestination(destination);
    return hasReachedDestination(quad_->getPose().getOrigin(), destination);
}

}


