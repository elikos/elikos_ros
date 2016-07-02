//
// Created by olivier on 28/06/16.
//

#include "MessageHandler.h"

#include "MovementState.h"
#include "StateMachine.h"

namespace ai
{

MovementState::MovementState(StateMachine* reference)
    : AbstractState(reference)
{
}

MovementState::~MovementState()
{
}

void MovementState::handleTargetSelection(TargetRobot* target, const QuadRobot& quad)
{
    tf::Vector3 destination = destination_;
    destination.setZ(FLIGHT_HEIGHT);
    MessageHandler::getInstance()->sendDestination(destination);
    if (hasReachedDestination(quad.getPose().getOrigin(), destination))
    {
        stateMachine_->setState(StateMachine::INTERACTION);
    }
}

}


