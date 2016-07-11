//
// Created by olivier on 08/07/16.
//

#include "MessageHandler.h"

#include "CommandQueue.h"
#include "FrontInteractionCommand.h"

namespace ai
{

FrontInteractionCommand::FrontInteractionCommand(QuadRobot* quad, TargetRobot* target)
    : AbstractCommand(quad, target)
{
}

FrontInteractionCommand::~FrontInteractionCommand()
{
}

bool FrontInteractionCommand::execute()
{
    tf::Vector3 destination = target_->getPose().getOrigin();
    destination += target_->getDirection() * FORWARD_OFFSET;
    MessageHandler::getInstance()->sendDestination(destination);
    return hasReachedDestination(quad_->getPose().getOrigin(), destination);
}

}
