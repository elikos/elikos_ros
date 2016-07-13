#include "Agent.h"
#include "PreventiveBehavior.h"
#include "AggressiveBehavior.h"
#include "ConsiderationTypes.h"

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
    if( behaviors_[PREVENTIVE]->isStateCritical(pipeline_.getArena())){
        behavior = behaviors_[PREVENTIVE].get();
    // Check if the current target has a good orientation
    } else if (behaviors_[AGGRESSIVE]->isStateCritical(pipeline_.getArena())) {
        behavior = behaviors_[AGGRESSIVE].get();
    // Interact with highest priority target
    } else {
        behavior = behaviors_[PREVENTIVE].get();
    }

    if (currentBehavior_ != behavior)
    {
        currentBehavior_ = behavior;
        currentBehavior_->generateCommands(pipeline_.getArena());
    }
}

void Agent::behave()
{
    resolveCurrentBehavior();
    currentBehavior_->behave();
}

void Agent::addConsideration(Consideration consideration)
{
    switch (consideration)
    {
        case TARGET_DESTINATION:
            pipeline_.addConsideration(std::unique_ptr<TargetDestination>(new TargetDestination()));
            break;
        case QUAD_DISTANCE:
            pipeline_.addConsideration(std::unique_ptr<QuadDistance>(new QuadDistance()));
            break;
        case CLUSTER_SIZE:
            pipeline_.addConsideration(std::unique_ptr<ClusterSize>(new ClusterSize()));
            break;
    }
}

void Agent::updateTargets(const elikos_ros::TargetRobotArray::ConstPtr& input)
{
    pipeline_.updateTargets(input);
}

void Agent::updateQuadRobot(const tf::Pose& pose)
{
    pipeline_.updateQuadRobot(pose);
}

};
