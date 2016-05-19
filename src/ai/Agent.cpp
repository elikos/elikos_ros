#include "Agent.h"
#include "StrategyTypes.h"

namespace ai
{

Agent::Agent()
{
   strategy_ = std::unique_ptr<FollowClosestTarget>(new FollowClosestTarget(quad_));
}

void Agent::behave()
{
    Robot* target = strategy_->findTargetSelection();

    //TODO: update the current target,


}


};
