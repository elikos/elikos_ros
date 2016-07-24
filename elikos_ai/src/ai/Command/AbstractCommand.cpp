#include "CommandQueue.h"

namespace ai
{

AbstractCommand::AbstractCommand(QuadRobot* quad, TargetRobot* target)
    : quad_(quad), target_(target)
{
}

AbstractCommand::~AbstractCommand()
{
}

bool AbstractCommand::hasReachedDestination(const tf::Vector3& currentPosition, const tf::Vector3& destination)
{
    double distance = tf::tfDistance(currentPosition, destination);
    
    return std::abs(distance) < 0.4;
}

}
