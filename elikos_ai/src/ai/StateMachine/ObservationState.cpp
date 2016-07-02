//
// Created by olivier on 28/06/16.
//

#include "MessageHandler.h"

#include "StateMachine.h"

#include "ObservationState.h"

namespace ai
{

const tf::Point ObservationState::OBSERVATION_POSITION{ 0.0, 0.0, 6.0 };

ObservationState::ObservationState(StateMachine* reference)
    : AbstractState(reference)
{
}

ObservationState::~ObservationState()
{
}

void ObservationState::handleTargetSelection(TargetRobot* target, const QuadRobot& quad)
{
    MessageHandler::getInstance()->sendDestination(OBSERVATION_POSITION);
    bool destionationReached = hasReachedDestination(quad.getPose().getOrigin(), OBSERVATION_POSITION);
    if (destionationReached)
    {
        stateMachine_->getState(StateMachine::MOVEMENT)->setDestination(target->getPose().getOrigin());
        stateMachine_->setState(StateMachine::MOVEMENT);
    }
}

}

