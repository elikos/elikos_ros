#include "MessageHandler.h"
#include "TargetRobot.h"
#include "QuadRobot.h"

#include "TakeOffCommand.h"

namespace ai
{

TakeOffCommand::TakeOffCommand(QuadRobot* quad, const tf::Point& destination)
    : AbstractCommand(quad, nullptr), destination_(destination)
{
}

TakeOffCommand::~TakeOffCommand()
{
}

void TakeOffCommand::execute()
{
    MessageHandler::getInstance()->sendDestination(destination_, CmdCode::TAKEOFF);
}

bool TakeOffCommand::isCommmandDone()
{
    bool has_reached_destination = hasReachedDestination(quad_->getPose().getOrigin(), destination_);
    return  has_reached_destination ||
            timer_.getElapsedS() > WAIT_TIME;
}

}


