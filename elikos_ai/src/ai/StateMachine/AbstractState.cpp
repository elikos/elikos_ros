#include "StateMachine.h"

namespace ai
{

AbstractState::AbstractState(StateMachine* stateMachine, QuadRobot* quad)
    : stateMachine_(stateMachine), quad_(quad)
{
}

AbstractState::~AbstractState()
{
}

void AbstractState::handlePriorityUpdate(TargetRobot* highestPriority)
{
    // do nothing by default
}


bool AbstractState::hasReachedDestination(const tf::Vector3& currentPosition, const tf::Vector3& destination)
{
    double distance = tf::tfDistance(currentPosition, destination);
    return std::abs(distance) < 0.1;
}

}
