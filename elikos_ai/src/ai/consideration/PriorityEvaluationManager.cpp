//
// Created by olivier on 27/06/16.
//

#include <thread>
#include "AbstractConsideration.h"
#include "PriorityEvaluationManager.h"
#include "AbstractArena.h"
#include "Configuration.h"

#include "ArenaA.h"

namespace ai
{

PriorityEvaluationManager::PriorityEvaluationManager(AbstractArena* arena, Configuration* config)
    : arena_(arena)
{
    //TODO: use the config here.
}

void PriorityEvaluationManager::updateTargets(const elikos_ros::TargetRobotArray::ConstPtr& input)
{
    const elikos_ros::TargetRobot* targets = input->targets.data();
    size_t n = input->targets.size();
    //std::thread th[n];

    for (int i = 0; i < n; ++i)
    {
        updateTarget(targets[i]);
        //th[i] = std::thread(&PriorityEvaluationManager::updateTarget, this, targets[i]);
        //th[i].join();
    }
}

void PriorityEvaluationManager::updateTarget(const elikos_ros::TargetRobot& targetUpdate)
{
    /*TargetRobot* target = arena_->updateTarget(targetUpdate);
    if (target != nullptr) {
        arena_->evaluateTargetOrientation(*target);
        evaluatePriority(*target);
    } */
}

void PriorityEvaluationManager::evaluatePriority(TargetRobot& target)
{
    for (int i = 0; i < considerations_.size(); i++)
    {
        considerations_[i]->evaluatePriority(arena_);
    }
}

void PriorityEvaluationManager::updateQuadRobot(const tf::Pose& pose)
{
    arena_->getQuad().setPose(pose);
}

}
