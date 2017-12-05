//
// Created by olivier on 28/06/16.
//

#include "MessageHandler.h"
#include "TargetRobot.h"
#include "QuadRobot.h"

#include "ObservationCommand.h"

namespace ai
{

ObservationCommand::ObservationCommand(QuadRobot* quad, TargetRobot* target)
    : AbstractCommand(quad, target)
{
    timer_.start();
    observation_position_.setX(0);
    observation_position_.setY(0);
    
    ros::NodeHandle nh;
    double research_altitude;
 	nh.getParam("/elikos_decisionmaking/research_altitude", research_altitude);
    observation_position_.setZ(research_altitude);
}

ObservationCommand::~ObservationCommand()
{
}

void ObservationCommand::execute()
{
    ai::MessageHandler::getInstance()->publishAiStateCommand("Observation command");
    MessageHandler::getInstance()->sendDestination(observation_position_, CmdCode::MOVE_TO_POINT);
}


bool ObservationCommand::isCommmandDone()
{
    return hasReachedDestination(quad_->getPose().getOrigin(), observation_position_);
}

}

