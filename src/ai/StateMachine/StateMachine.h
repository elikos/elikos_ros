#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <memory>
#include "State.h"



class StateMachine
{
public:

    StateMachine();
    ~StateMachine();

private:
    std::unique_ptr<State> currentState_;

}; 

#endif /// STATE_MACHINE_H
