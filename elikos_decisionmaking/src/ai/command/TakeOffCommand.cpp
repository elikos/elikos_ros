#include "MessageHandler.h"
#include "TargetRobot.h"
#include "QuadRobot.h"

#include "TakeOffCommand.h"

namespace ai
{

TakeOffCommand::TakeOffCommand(QuadRobot* quad, tf::TransformListener* tf_listener)
    : AbstractCommand(quad, nullptr), tf_listener_(tf_listener)
{
}

TakeOffCommand::~TakeOffCommand()
{
}

void TakeOffCommand::execute()
{
   ai::MessageHandler::getInstance()->publishAiStateCommand("Take off command");

   ros::NodeHandle nh;
   double takeoff_altitude;
   nh.getParam("/elikos_decisionmaking/takeoff_altitude", takeoff_altitude);

   tf::StampedTransform currentPosition;
   try {
        tf_listener_->lookupTransform("elikos_arena_origin", "elikos_fcu", ros::Time(0), currentPosition);
    } catch (tf::TransformException e) {
        ROS_ERROR("TAKEOFF : %s",e.what());
    }
    destination_.setX(currentPosition.getOrigin().x());
    destination_.setY(currentPosition.getOrigin().y());
    destination_.setZ(takeoff_altitude);

    MessageHandler::getInstance()->sendDestination(destination_, CmdCode::TAKEOFF);
}

bool TakeOffCommand::isCommmandDone()
{
    bool has_reached_destination = hasReachedDestination(quad_->getPose().getOrigin(), destination_);
    return  has_reached_destination ||
            timer_.getElapsedS() > WAIT_TIME;
}

}


