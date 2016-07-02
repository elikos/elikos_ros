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

    static const tf::Point OBSERVATION_POSITION;


    ObservationState(StateMachine* stateMachine, QuadRobot* quad);
    virtual ~ObservationState();

    virtual void handlePriorityUpdate(TargetRobot* highestPriorityTarget);
    virtual void behave();

private:
    ObservationState() = delete;
};

}

#endif //ELIKOS_AI_OBSERVATIONSTATE_H
