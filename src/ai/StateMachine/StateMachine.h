#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <memory>
#include "AbstractState.h"

enum State
{
    DEFAULT
};


class StateMachine
{
public:

    StateMachine();
    ~StateMachine();

    void setState(State state);

private:
    std::unique_ptr<AbstractState> currentState_;

}; 

#endif /// STATE_MACHINE_H
