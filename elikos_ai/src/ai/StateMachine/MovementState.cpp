//
// Created by olivier on 28/06/16.
//

#include "MessageHandler.h"

#include "MovementState.h"
#include "StateMachine.h"

namespace ai
{

MovementState::MovementState(StateMachine* stateMachine, QuadRobot* quad)
    : AbstractState(stateMachine, quad)
{
}

MovementState::~MovementState()
{
}

void MovementState::handlePriorityUpdate(TargetRobot* highestPriorityTarget)
{
    target_ = highestPriorityTarget;
}

void MovementState::behave()
{
    tf::Vector3 destination = target_->getPose().getOrigin();
    destination.setZ(FLIGHT_HEIGHT);
    MessageHandler::getInstance()->sendDestination(destination);
    if (hasReachedDestination(quad_->getPose().getOrigin(), destination))
    {
        stateMachine_->setState(StateMachine::INTERACTION, target_);
    }
}

}


