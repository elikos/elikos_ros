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

    StateMachine();
    ~StateMachine() = default;

    inline void setState(EnumState state);
    void handleTargetSelection(TargetRobot* target, const QuadRobot& quad);

private:
    using StatePtr = std::unique_ptr<AbstractState>;
    std::unordered_map<int, StatePtr> states_;
    AbstractState* currentState_;
    //TODO: use command pattern with a q for handling more complexe trajectories.
};

inline void StateMachine::setState(EnumState state)
{
    currentState_ = states_[state].get();
}

}

#endif /// STATE_MACHINE_H
