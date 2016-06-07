#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <memory>
#include "State.h"



class StateMachine
{
public:
    enum STATE_ENUM
    {
        GOING_FOR_TARGET,
        DODGING_OBSTACLE,

    };

    StateMachine();
    ~StateMachine();

private:
    std::unique_ptr<State> currentState_;

}; 

#endif /// STATE_MACHINE_H
