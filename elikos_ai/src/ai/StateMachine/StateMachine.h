#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <memory>
#include <unordered_map>
#include <queue>

#include "Robot/TargetRobot.h"
#include "Robot/QuadRobot.h"
#include "AbstractState.h"
#include "Command.h"

namespace ai
{

class StateMachine
{
public:
    enum EnumState
    {
        OBSERVATION,
        MOVEMENT,
        INTERACTION
    };

    StateMachine(QuadRobot& quad);
    ~StateMachine() = default;

    inline void setState(EnumState state, TargetRobot* target);
    inline AbstractState* getState(EnumState state);

    void updatePriorityTarget(TargetRobot* robot);
    void behave();

private:
    using StatePtr = std::unique_ptr<AbstractState>;
    std::unordered_map<int, StatePtr> states_;
    QuadRobot& quad_;

    AbstractState* currentState_;
};

inline void StateMachine::setState(EnumState state, TargetRobot* target)
{
    states_[state]->setTarget(target);
    currentState_ = states_[state].get();

}

inline AbstractState* StateMachine::getState(EnumState state)
{
    return states_[state].get();
}

}

#endif /// STATE_MACHINE_H
