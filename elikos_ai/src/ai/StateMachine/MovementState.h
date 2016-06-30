#ifndef AI_MOVEMENT_STATE_H
#define AI_MOVEMENT_STATE_H

#include "RobotTypes.h"

#include "AbstractState.h"

namespace ai
{

class StateMachine;

class MovementState : public AbstractState
{
public:

    static constexpr double FLIGHT_HEIGHT{ 2.0 };

    MovementState(StateMachine* reference);
    virtual ~MovementState();
    virtual void handleTargetSelection(TargetRobot* target, const QuadRobot& quad);

private:
    MovementState() = delete;
};

}

#endif // AI_MOVEMENT_STATE_H
