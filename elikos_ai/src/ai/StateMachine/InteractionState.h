//
// Created by olivier on 28/06/16.
//

#ifndef AI_INTERACTION_STATE_H
#define AI_INTERACTION_STATE_H

#include "Robot/RobotTypes.h"

#include "AbstractState.h"

namespace ai
{

class StateMachine;

class InteractionState : public AbstractState
{
public:

    static constexpr double HEIGHT_OFFSET{ 0.0 };

    InteractionState(StateMachine* stateMachine, QuadRobot* quad);

    virtual void handlePriorityUpdate(TargetRobot* highestPriorityTarget);

    virtual void behave();
    virtual ~InteractionState();

private:
    InteractionState() = delete;

};

}

#endif //ELIKOS_AI_INTERACTIONSTATE_H

