//
// Created by olivier on 27/06/16.
//

#include <thread>
#include "AbstractConsideration.h"
#include "PriorityEvaluationPipeline.h"
#include "ArenaA.h"

namespace ai
{

PriorityEvaluationPipeline::PriorityEvaluationPipeline()
{
    arena_ = std::unique_ptr<ArenaA>(new ArenaA());
}

void PriorityEvaluationPipeline::addConsideration(std::unique_ptr<AbstractConsideration> consideration)
{
    considerations_.push_back(std::move(consideration));
}

void PriorityEvaluationPipeline::updateTargets(const elikos_ros::TargetRobotArray::ConstPtr& input)
{
    const elikos_ros::TargetRobot* targets = input->targets.data();
    size_t n = input->targets.size();
    //std::thread th[n];

    arena_->prepareUpdate();
    for (int i = 0; i < n; ++i)
    {
        updateTarget(targets[i]);
        //th[i] = std::thread(&PriorityEvaluationPipeline::updateTarget, this, targets[i]);
        //th[i].join();
    }
}

void PriorityEvaluationPipeline::updateTarget(const elikos_ros::TargetRobot& targetUpdate)
{
    // TODO: Remove the index and implement a way to match the target update in arena.
    TargetRobot* target = arena_->updateTarget(targetUpdate);
    if (target != nullptr) {
        arena_->evaluateTargetOrientation(*target);
        evaluatePriority(*target);
    }
}

void PriorityEvaluationPipeline::evaluatePriority(TargetRobot& target)
{
    for (int i = 0; i < considerations_.size(); i++)
    {
        considerations_[i]->evaluatePriority(target, arena_.get());
    }
}

}
