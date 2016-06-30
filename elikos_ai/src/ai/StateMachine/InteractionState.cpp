//
// Created by olivier on 28/06/16.
//

#include "MessageHandler.h"

#include "InteractionState.h"
#include "StateMachine.h"

namespace ai
{

InteractionState::InteractionState(StateMachine* reference)
    : AbstractState(reference)
{
}

InteractionState::~InteractionState()
{
}

void InteractionState::handleTargetSelection(TargetRobot* target, const QuadRobot& quad)
{
    tf::Vector3 destination = target->getPose().getOrigin();
    MessageHandler::getInstance()->sendDestination(destination);
    if (hasReachedDestination(quad.getPose().getOrigin(), destination))
    {
        stateMachine_->setState(StateMachine::MOVEMENT);
    }
}

}
