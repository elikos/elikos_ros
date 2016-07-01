#include "Agent.h"
#include "MessageHandler.h"

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
    delete instance_;
    instance_ = nullptr;
}

void Agent::behave()
{
    TargetRobot* target = pipeline_.evaluateTargetSelection(quad_);
    stateMachine_.handleTargetSelection(target, quad_);

}


};
