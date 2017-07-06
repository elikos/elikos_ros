//
// Created by olivier on 28/06/16.
//

#include "MessageHandler.h"
#include "TargetRobot.h"
#include "QuadRobot.h"

#include "FollowCommand.h"

namespace ai
{

FollowCommand::FollowCommand(QuadRobot* quad, TargetRobot* target)
    : AbstractCommand(quad, target)
{
}

FollowCommand::~FollowCommand()
{
}

void FollowCommand::execute()
{
    tf::Vector3 destination = target_->getPose().getOrigin();
    destination.setZ(FLIGHT_HEIGHT);
    MessageHandler::getInstance()->sendDestination(destination, CmdCode::MOVE_TO_POINT);
}

bool FollowCommand::isCommmandDone()
{
    tf::Vector3 destination = target_->getPose().getOrigin();
    destination.setZ(FLIGHT_HEIGHT);
    return hasReachedDestination(quad_->getPose().getOrigin(), destination) ||
            timer_.getElapsedS() > WAIT_TIME;
}

}


