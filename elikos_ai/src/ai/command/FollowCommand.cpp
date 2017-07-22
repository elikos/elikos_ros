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
    ros::NodeHandle nh;
    nh.getParam("/elikos_ai/research_altitude", flight_height_);
}

FollowCommand::~FollowCommand()
{
}

void FollowCommand::execute()
{
    ai::MessageHandler::getInstance()->publishAiStateCommand("Follow target command");
    tf::Vector3 destination = target_->getPose().getOrigin();
    destination.setZ(flight_height_);
    MessageHandler::getInstance()->sendDestination(destination, CmdCode::MOVE_TO_POINT);
}

bool FollowCommand::isCommmandDone()
{
    tf::Vector3 destination = target_->getPose().getOrigin();
    destination.setZ(flight_height_);
    return hasReachedDestination(quad_->getPose().getOrigin(), destination) ||
            timer_.getElapsedS() > WAIT_TIME;
}

}


