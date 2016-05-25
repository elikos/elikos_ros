#include "Agent.h"
#include "StrategyTypes.h"

namespace ai
{

Agent* Agent::instance_ = nullptr;

Agent* Agent::getInstance()
{
    if (instance_ == nullptr)
    {
        instance_ = new Agent();
    }
    return instance_;
}

Agent::Agent()
{
   strategy_ = std::unique_ptr<FollowClosestTarget>(new FollowClosestTarget(quad_));
}

void Agent::freeInstance()
{

}

void Agent::behave()
{
    Robot* target = strategy_->findTargetSelection();

    //TODO: update the current target,

}


};
