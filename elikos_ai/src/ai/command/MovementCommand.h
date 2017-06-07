#ifndef AI_MOVEMENT_COMMAND_H
#define AI_MOVEMENT_COMMAND_H

#include "AbstractCommand.h"

namespace ai
{

class CommandQueue;

class MovementCommand : public AbstractCommand
{
public:
    static constexpr double FLIGHT_HEIGHT{ 2.0 };
    static constexpr double WAIT_TIME { 5.0 };

    MovementCommand(QuadRobot* quad, const tf::Point& destination);
    virtual ~MovementCommand();
    virtual bool isCommmandDone();
    virtual void execute();

private:
    MovementCommand() = delete;
    tf::Point destination_;
    bool is_takeoff_;
};

}

#endif // AI_MOVEMENT_COMMAND_H
