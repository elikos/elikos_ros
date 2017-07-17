//
// Created by olivier on 28/06/16.
//

#include "MessageHandler.h"
#include "TargetRobot.h"
#include "QuadRobot.h"

#include "TopInteractionCommand.h"


namespace ai
{

TopInteractionCommand::TopInteractionCommand(QuadRobot* quad, TargetRobot* target)
    : AbstractCommand(quad, target)
{
}



TopInteractionCommand::~TopInteractionCommand()
{
}

void TopInteractionCommand::execute()
{
    tf::Vector3 destination = target_->getPose().getOrigin();
    destination.setZ(0.0);
    MessageHandler::getInstance()->sendDestination(destination, CmdCode::TOP_INTERACTION);
}


bool TopInteractionCommand::isCommmandDone()
{
    tf::Vector3 destination = target_->getPose().getOrigin();
    destination.setZ(0.0);
    return hasReachedDestination(quad_->getPose().getOrigin(), destination);
}

}
