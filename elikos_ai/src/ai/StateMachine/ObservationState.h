//
// Created by olivier on 28/06/16.
//

#ifndef ELIKOS_AI_OBSERVATION_STATE_H
#define ELIKOS_AI_OBSERVATION_STATE_H

#include "Robot/RobotTypes.h"

#include "AbstractState.h"

namespace ai
{

class StateMachine;

class ObservationState : public AbstractState
{
public:
    ObservationState(StateMachine* reference);
    virtual ~ObservationState();
    virtual void handleTargetSelection(TargetRobot* target, const QuadRobot& quad);

private:
    ObservationState() = delete;
};

}

#endif //ELIKOS_AI_OBSERVATIONSTATE_H
