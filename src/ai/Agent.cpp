#include "Agent.h"
#include "StrategyTypes.h"

namespace ai
{

Agent::Agent()
{
   strategy_ = std::unique_ptr<FollowClosestTarget>(new FollowClosestTarget());
}

void Agent::updateTarget(const int& id, const tf::Vector3& position, const tf::Quaternion& orientation)
{

}

void Agent::updateObstacle(const int& id, const tf::Vector3& position, const tf::Quaternion& orientation)
{

}

void Agent::updateMAV(const tf::Vector3& position, const tf::Quaternion& orientation)
{

}

}

void takeADecision();
