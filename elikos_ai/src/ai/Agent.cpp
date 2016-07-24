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
    behaviors_[PREVENTIVE] = std::unique_ptr<PreventiveBehavior>(new PreventiveBehavior(pipeline_.getArena()));
    behaviors_[AGGRESSIVE] = std::unique_ptr<AggressiveBehavior>(new AggressiveBehavior(pipeline_.getArena()));
    currentBehavior_ = behaviors_[AGGRESSIVE].get();
    updateTimer_.start();
}

void Agent::freeInstance()
{
    delete instance_;
    instance_ = nullptr;
}

AbstractBehavior* Agent::resolveCurrentBehavior()
{
    // Check for robots that are about to go out of the arena first
    int preventiveStateLevel = behaviors_[PREVENTIVE]->resolveCurrentStateLevel();
    int aggressiveStateLevel = behaviors_[AGGRESSIVE]->resolveCurrentStateLevel();
    AbstractBehavior* behavior = behaviors_[AGGRESSIVE].get();

    if (preventiveStateLevel > aggressiveStateLevel) {
        behavior = behaviors_[PREVENTIVE].get();
    }
    return behavior;
}

void Agent::behave()
{
    currentBehavior_ = resolveCurrentBehavior();
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
    updateTimer_.reset();
    pipeline_.updateTargets(input);
}

void Agent::updateQuadRobot(const tf::Pose& pose)
{
    pipeline_.updateQuadRobot(pose);
}

void Agent::checkForTargetUpdate()
{
    if (updateTimer_.getElapsedS() > 2.0 ) {
        currentBehavior_->searchForTargets();
    }
}

};
