//
// Created by olivier on 08/07/16.
//

#include "MessageHandler.h"
#include "TargetRobot.h"
#include "QuadRobot.h"

#include "FrontInteractionCommand.h"

namespace ai
{

const double FrontInteractionCommand::FORWARD_OFFSET = 5.0;
const double FrontInteractionCommand::WAIT_TIME = 2.0;

FrontInteractionCommand::FrontInteractionCommand(QuadRobot* quad, TargetRobot* target)
    : AbstractCommand(quad, target)
{
}

FrontInteractionCommand::~FrontInteractionCommand()
{
}

void FrontInteractionCommand::execute()
{
    tf::Vector3 destination = target_->getPose().getOrigin();
    destination += (target_->getDirection() * FORWARD_OFFSET);
    MessageHandler::getInstance()->sendDestination(destination);
    if (hasReachedDestination(quad_->getPose().getOrigin(), destination) && !timer_.isStarted()) {
        timer_.start();
    }
}

bool FrontInteractionCommand::isCommmandDone()
{
    return (timer_.getElapsedS() > WAIT_TIME);
}

}
