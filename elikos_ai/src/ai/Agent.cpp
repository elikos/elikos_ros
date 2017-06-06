#include "Agent.h"
#include "PreventiveBehavior.h"
#include "AggressiveBehavior.h"
#include "ResearchBehavior.h"
#include "ConsiderationTypes.h"
#include "ArenaA.h"
#include "Configuration.h"

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

void Agent::init()
{
    arena_ = std::unique_ptr<ArenaA>(new ArenaA());
    behaviorManager_ = std::unique_ptr<BehaviorManager>(new BehaviorManager(arena_.get()));
    priorityManager_ = std::unique_ptr<PriorityEvaluationManager>(new PriorityEvaluationManager(arena_.get()));
}

void Agent::behave()
{
    behaviorManager_->behave();
}

void Agent::updateTargets(const elikos_ros::TargetRobotArray::ConstPtr& input)
{
    arena_->updateTargets(input);
}

void Agent::updateQuadRobot(const tf::Pose& pose)
{
    priorityManager_->updateQuadRobot(pose);
}

};
