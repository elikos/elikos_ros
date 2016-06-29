#include "Agent.h"
#include "MessageHandler.h"

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
}

void Agent::freeInstance()
{

}

void Agent::behave()
{
    Robot* target = pipeline_.evaluateTargetSelection();
    //TODO: The state machine should be sending the destinations.
    MessageHandler::getInstance()->sendDestination(target->getPose().getOrigin());
}


};
