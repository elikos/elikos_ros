#ifndef AI_MOVEMENT_STATE_H
#define AI_MOVEMENT_STATE_H

#include "Robot/RobotTypes.h"

#include "AbstractState.h"

namespace ai
{

class StateMachine;

class MovementState : public AbstractState
{
public:

    static constexpr double FLIGHT_HEIGHT{ 2.0 };

    MovementState(StateMachine* stateMachine, QuadRobot* quad);
    virtual ~MovementState();
    virtual void handlePriorityUpdate(TargetRobot* highestPriorityTarget);
    virtual void behave();

private:
    MovementState() = delete;
};

}

#endif // AI_MOVEMENT_STATE_H
