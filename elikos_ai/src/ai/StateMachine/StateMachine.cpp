#include "StateMachine.h"
#include "StateTypes.h"

namespace ai
{

StateMachine::StateMachine()
{
    states_.insert(std::pair<int, StatePtr>(OBSERVATION, StatePtr(new ObservationState(this))));
    states_.insert(std::pair<int, StatePtr>(MOVEMENT, StatePtr(new MovementState(this))));
    states_.insert(std::pair<int, StatePtr>(INTERACTION, StatePtr(new InteractionState(this))));
    currentState_ = states_[MOVEMENT].get();
}

void StateMachine::handleTargetSelection(TargetRobot* target, const QuadRobot& quad)
{
    currentState_->handleTargetSelection(target, quad);
}

}
