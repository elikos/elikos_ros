#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <memory>
#include "AbstractState.h"

namespace ai
{

class StateMachine
{
public:
    StateMachine() = default;
    ~StateMachine() = default;

private:
    std::unique_ptr<AbstractState> currentState_;
};

}

#endif /// STATE_MACHINE_H
