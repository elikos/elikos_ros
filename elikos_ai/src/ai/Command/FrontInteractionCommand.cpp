//
// Created by olivier on 08/07/16.
//

#include "MessageHandler.h"
#include "TargetRobot.h"
#include "QuadRobot.h"

#include "FrontInteractionCommand.h"

namespace ai
{

const double FrontInteractionCommand::FORWARD_OFFSET = 0.5;

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
