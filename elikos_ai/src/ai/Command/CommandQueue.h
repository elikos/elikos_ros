#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <memory>
#include <unordered_map>
#include <queue>

#include "Robot/TargetRobot.h"
#include "Robot/QuadRobot.h"
#include "AbstractCommand.h"
#include <deque>

namespace ai
{

class CommandQueue
{
public:

    CommandQueue() = default;
    ~CommandQueue() = default;

    bool empty();
    void push(std::unique_ptr<AbstractCommand> command);
    void pop();
    AbstractCommand* back();
    AbstractCommand* front();
    void clear();

private:
    std::deque<std::unique_ptr<AbstractCommand>> q_;

};

}

#endif /// STATE_MACHINE_H
