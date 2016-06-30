#include "StateMachine.h"

namespace ai
{

AbstractState::AbstractState(StateMachine* reference)
    : stateMachine_(reference)
{
}

AbstractState::~AbstractState()
{
}

bool AbstractState::hasReachedDestination(const tf::Vector3& currentPosition, const tf::Vector3& destination)
{
    double distance = tf::tfDistance(currentPosition, destination);
    return std::abs(distance) < 0.1;
}

}
