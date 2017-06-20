//
// Created by olivier on 28/06/16.
//

#include "MessageHandler.h"
#include "TargetRobot.h"
#include "QuadRobot.h"

#include "MovementCommand.h"

namespace ai
{

MovementCommand::MovementCommand(QuadRobot* quad, const tf::Point& destination)
    : AbstractCommand(quad, nullptr), destination_(destination)
{
}

MovementCommand::~MovementCommand()
{
}

void MovementCommand::execute()
{
    MessageHandler::getInstance()->sendDestination(destination_, CmdCode::MOVE_TO_POINT);
}

bool MovementCommand::isCommmandDone()
{
    bool has_reached_destination = hasReachedDestination(quad_->getPose().getOrigin(), destination_);
    return  has_reached_destination ||
            timer_.getElapsedS() > WAIT_TIME;
}

}


