#include "CommandQueue.h"

namespace ai
{

AbstractCommand::AbstractCommand(QuadRobot* quad, TargetRobot* target)
    : quad_(quad), target_(target)
{
    ros::NodeHandle nh_;
    has_reach_destination_threshold_ = 0.2;
    nh_.getParam("/elikos_ai/has_reach_destination_threshold", has_reach_destination_threshold_);
}

AbstractCommand::~AbstractCommand()
{
}

bool AbstractCommand::hasReachedDestination(const tf::Vector3& currentPosition, const tf::Vector3& destination)
{
    double distance;
    if(destination.getZ() < -0.5)
    {
      tf::Vector3 groundPosition = destination;
      groundPosition.setZ(0);
      distance = tf::tfDistance(currentPosition, groundPosition);
    }
    else
    {
      distance = tf::tfDistance(currentPosition, destination);
    }
    return std::abs(distance) < has_reach_destination_threshold_;
}

}
