//
// Created by olivier on 28/06/16.
//

#include "MessageHandler.h"
#include "TargetRobot.h"
#include "QuadRobot.h"

#include "MovementCommand.h"

namespace ai
{

MovementCommand::MovementCommand(QuadRobot* quad, TargetRobot* target)
    : AbstractCommand(quad, target)
{
}

MovementCommand::~MovementCommand()
{
}

void MovementCommand::execute()
{
    tf::Vector3 destination = target_->getPose().getOrigin();
    destination.setZ(FLIGHT_HEIGHT);
    MessageHandler::getInstance()->sendDestination(destination);
}

bool MovementCommand::isCommmandDone()
{
    tf::Vector3 destination = target_->getPose().getOrigin();
    destination.setZ(FLIGHT_HEIGHT);
    return hasReachedDestination(quad_->getPose().getOrigin(), destination) ||
            timer_.getElapsedS() > WAIT_TIME;
}

}


