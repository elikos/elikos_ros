//
// Created by olivier on 28/06/16.
//

#include "MessageHandler.h"

#include "InteractionState.h"
#include "StateMachine.h"

namespace ai
{

InteractionState::InteractionState(StateMachine* stateMachine, QuadRobot* quad)
    : AbstractState(stateMachine, quad)
{
}

InteractionState::~InteractionState()
{
}

void InteractionState::behave()
{
    tf::Vector3 destination = target_->getPose().getOrigin();
    MessageHandler::getInstance()->sendDestination(destination);
    if (hasReachedDestination(quad_->getPose().getOrigin(), destination))
    {
        stateMachine_->setState(StateMachine::OBSERVATION, target_);
    }
}

}
