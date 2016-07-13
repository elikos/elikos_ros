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
    std::thread th[n];

    for (int i = 0; i < n; ++i)
    {
        th[i] = std::thread(&PriorityEvaluationPipeline::updateTarget, this, targets[i], i);
    }

    for (int i = 0; i < n; ++i)
    {
        th[i].join();
    }
}

void PriorityEvaluationPipeline::updateTarget(const elikos_ros::TargetRobot& targetUpdate, int i)
{
    std::vector<TargetRobot>& targets = arena_->getTargets();
    tf::Pose pose;
    tf::poseMsgToTF(targetUpdate.poseOrigin.pose, pose);
    targets[i].setPose(pose);
    targets[i].setId(targetUpdate.id);
    targets[i].setColor(targetUpdate.color);
    arena_->evaluateTargetOrientation(targets[i]);
    evaluatePriority(targets[i]);
}

void PriorityEvaluationPipeline::evaluatePriority(TargetRobot& target)
{
    for (int j = 0; j < considerations_.size(); j++)
    {
        considerations_[j]->evaluatePriority(target, arena_.get());
    }
}

TargetRobot* PriorityEvaluationPipeline::findHighestPriorityTarget()
{
    return arena_->findHighestPriorityTarget();
}

}
