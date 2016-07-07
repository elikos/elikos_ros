#include "StateMachine.h"
#include "StateTypes.h"

namespace ai
{

StateMachine::StateMachine(QuadRobot& quad)
    : quad_(quad)
{
    states_.insert(std::pair<int, StatePtr>(OBSERVATION, StatePtr(new ObservationState(this, &quad))));
    states_.insert(std::pair<int, StatePtr>(MOVEMENT, StatePtr(new MovementState(this, &quad))));
    states_.insert(std::pair<int, StatePtr>(INTERACTION, StatePtr(new InteractionState(this, &quad))));
    currentState_ = states_[OBSERVATION].get();
}

void StateMachine::behave()
{
    currentState_->behave();
}

void  StateMachine::updatePriorityTarget(TargetRobot* robot)
{
    currentState_->handlePriorityUpdate(robot);
}

}
