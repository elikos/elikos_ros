//
// Created by olivier on 28/06/16.
//

#include "MessageHandler.h"

#include "TopInteractionCommand.h"
#include "CommandQueue.h"

namespace ai
{

TopInteractionCommand::TopInteractionCommand(QuadRobot* quad, TargetRobot* target)
    : AbstractCommand(quad, target)
{
}



TopInteractionCommand::~TopInteractionCommand()
{
}

bool TopInteractionCommand::execute()
{
    tf::Vector3 destination = target_->getPose().getOrigin();
    MessageHandler::getInstance()->sendDestination(destination);
    return hasReachedDestination(quad_->getPose().getOrigin(), destination);
}

}
