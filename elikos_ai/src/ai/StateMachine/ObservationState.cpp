//
// Created by olivier on 28/06/16.
//

#include "ObservationState.h"

namespace ai
{

ObservationState::ObservationState(StateMachine* reference)
    : AbstractState(reference)
{
}

ObservationState::~ObservationState()
{
}

void ObservationState::handleTargetSelection(TargetRobot* target, const QuadRobot& quad)
{

}

}

