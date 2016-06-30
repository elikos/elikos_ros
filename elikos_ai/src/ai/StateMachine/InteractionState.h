//
// Created by olivier on 28/06/16.
//

#ifndef AI_INTERACTION_STATE_H
#define AI_INTERACTION_STATE_H

#include "RobotTypes.h"

#include "AbstractState.h"

namespace ai
{

class StateMachine;

class InteractionState : public AbstractState
{
public:

    static constexpr double HEIGHT_OFFSET{ 0.0 };

    InteractionState(StateMachine* reference);

    virtual ~InteractionState();
    virtual void handleTargetSelection(TargetRobot* target, const QuadRobot& quad);

private:
    InteractionState() = delete;

};

}

#endif //ELIKOS_AI_INTERACTIONSTATE_H

