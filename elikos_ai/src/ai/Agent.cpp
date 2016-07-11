#include "Agent.h"
#include "MessageHandler.h"
#include "PreventiveBehavior.h"
#include "AggressiveBehavior.h"

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
    behaviors_[PREVENTIVE] = std::unique_ptr<PreventiveBehavior>(new PreventiveBehavior());
    behaviors_[AGGRESSIVE] = std::unique_ptr<AggressiveBehavior>(new AggressiveBehavior());
    currentBehavior_ = behaviors_[AGGRESSIVE].get();
}

void Agent::freeInstance()
{
    delete instance_;
    instance_ = nullptr;
}

AbstractBehavior* Agent::resolveCurrentBehavior()
{
    AbstractBehavior* behavior = nullptr;
    // Check for robots that are about to go out of the arena first
    if( behaviors_[PREVENTIVE]->isContextCritical(context_)) {
        behavior = behaviors_[PREVENTIVE].get();
    // Check if the current target has a good orientation
    } else if (behaviors_[AGGRESSIVE]->isContextCritical(context_)) {
        behavior = behaviors_[AGGRESSIVE].get();
    // Interact with highest priority target
    } else {
        behavior = behaviors_[PREVENTIVE].get();
    }

    if (currentBehavior_ != behavior)
    {
        currentBehavior_ = behavior;
        currentBehavior_->generateCommands(context_);
    }
}

void Agent::behave()
{
}

void Agent::updateTargets(const elikos_ros::TargetRobotArray::ConstPtr& input)
{
    context_.updateTargets(input);
    pipeline_.evaluatePriority(context_);
    resolveCurrentBehavior();
}

};
