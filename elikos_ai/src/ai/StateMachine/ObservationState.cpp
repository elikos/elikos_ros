//
// Created by olivier on 28/06/16.
//

#include "MessageHandler.h"

#include "StateMachine.h"

#include "ObservationState.h"

namespace ai
{

const tf::Point ObservationState::OBSERVATION_POSITION{ 0.0, 0.0, 3.0 };

ObservationState::ObservationState(StateMachine* stateMachine, QuadRobot* quad)
    : AbstractState(stateMachine, quad)
{
}

ObservationState::~ObservationState()
{
}

void ObservationState::handlePriorityUpdate(TargetRobot* highestPriority)
{
    target_ = highestPriority;
}

void ObservationState::behave()
{
    MessageHandler::getInstance()->sendDestination(OBSERVATION_POSITION);
    bool destionationReached = hasReachedDestination(quad_->getPose().getOrigin(), OBSERVATION_POSITION);
    if (destionationReached)
    {
        stateMachine_->setState(StateMachine::MOVEMENT, target_);
    }
}

}

