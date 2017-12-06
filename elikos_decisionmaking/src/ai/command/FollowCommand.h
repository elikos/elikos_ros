#ifndef AI_FOLLOW_COMMAND_H
#define AI_FOLLOW_COMMAND_H

#include "AbstractCommand.h"

namespace ai
{

class CommandQueue;

class FollowCommand : public AbstractCommand
{
public:
    static constexpr double FLIGHT_HEIGHT{ 2.0 };
    static constexpr double WAIT_TIME { 5.0 };

    FollowCommand(QuadRobot* quad, TargetRobot* target);
    virtual ~FollowCommand();
    virtual bool isCommmandDone();
    virtual void execute();

private:
    FollowCommand() = delete;
    double flight_height_;
};

}

#endif // AI_FOLLOW_COMMAND_H
