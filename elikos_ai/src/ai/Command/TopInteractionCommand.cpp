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
    MessageHandler::getInstance()->sendDestination(destination);
    if (hasReachedDestination(quad_->getPose().getOrigin(), destination) && !timer_.isStarted()) {
        timer_.start();
    }
}

bool TopInteractionCommand::isCommmandDone()
{
    double test = timer_.getElapsedS();
    return (test > WAIT_TIME);
    //return (timer_.getElapsedS() > WAIT_TIME);
}

}
