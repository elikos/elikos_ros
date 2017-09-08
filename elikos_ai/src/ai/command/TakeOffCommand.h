#ifndef AI_TakeOff_COMMAND_H
#define AI_TakeOff_COMMAND_H

#include "AbstractCommand.h"

namespace ai
{

class CommandQueue;

class TakeOffCommand : public AbstractCommand
{
public:

    static constexpr double WAIT_TIME { 5.0 };
    
    TakeOffCommand(QuadRobot* quad, tf::TransformListener* tf_listener);
    virtual ~TakeOffCommand();
    virtual bool isCommmandDone();
    virtual void execute();

private:
    TakeOffCommand() = delete;
    tf::Point destination_;

    tf::TransformListener* tf_listener_;
};

}

#endif // AI_TakeOff_COMMAND_H
