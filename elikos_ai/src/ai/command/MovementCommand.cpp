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
    : AbstractCommand(quad, nullptr), destination_(destination), is_takeoff_(true)
{
}

MovementCommand::~MovementCommand()
{
}

void MovementCommand::execute()
{
    if (is_takeoff_)
    {
        MessageHandler::getInstance()->sendDestination(destination_, CmdCode::TAKEOFF);
    }
    else
    {
        MessageHandler::getInstance()->sendDestination(destination_, CmdCode::MOVE_TO_POINT);
    }
}

bool MovementCommand::isCommmandDone()
{
    bool has_reached_destination = hasReachedDestination(quad_->getPose().getOrigin(), destination_);
    if (is_takeoff_ && has_reached_destination) is_takeoff_ = false;
    return  has_reached_destination ||
            timer_.getElapsedS() > WAIT_TIME;
}

}


