//
// Created by olivier on 28/06/16.
//

#include "MessageHandler.h"
#include "TargetRobot.h"
#include "QuadRobot.h"

#include "ObservationCommand.h"

namespace ai
{

const tf::Point ObservationCommand::OBSERVATION_POSITION{ 0.0, 0.0, 3.0 };

ObservationCommand::ObservationCommand(QuadRobot* quad, TargetRobot* target)
    : AbstractCommand(quad, target)
{
    timer_.start();
}

ObservationCommand::~ObservationCommand()
{
}

void ObservationCommand::execute()
{
    MessageHandler::getInstance()->sendDestination(OBSERVATION_POSITION);
}


bool ObservationCommand::isCommmandDone()
{
    return hasReachedDestination(quad_->getPose().getOrigin(), OBSERVATION_POSITION);
}

}

