//
// Created by olivier on 10/07/16.
//

#include <thread>
#include "ArenaA.h"

#include "Context.h"

namespace ai
{

Context::Context()
{
    arena_ = std::unique_ptr<ArenaA>(new ArenaA());
    arena_->populateTargets(targets_);
}

void Context::updateTargets(const elikos_ros::TargetRobotArray::ConstPtr& input)
{
    std::thread th[targets_.size()];
    size_t n = input->targets.size();
    const elikos_ros::TargetRobot* targets = input->targets.data();
    for (size_t i = 0; i < n; ++i)
    {
        th[i] = std::thread(updateTarget, targets[i], i, this);
    }

    for (int i = 0; i < n; ++i)
    {
        th[i].join();
    }
}

void Context::updateTarget(const elikos_ros::TargetRobot& targetUpdate, int i)
{
    tf::Pose pose;
    tf::poseMsgToTF(targetUpdate.poseOrigin.pose, pose);
    targets_[i].setPose(pose);
    targets_[i].setId(targetUpdate.id);
    targets_[i].setColor(targetUpdate.color);
    arena_->evaluateTargetOrientation(targets_[i]);
}

TargetRobot* Context::findHighestPriorityTarget()
{
    TargetRobot* target = &targets_[0];
    double highestPriority = targets_[0].getPriority();
    for ( int i = 1; i < targets_.size(); ++i)
    {
        if (targets_[i].getPriority() > highestPriority)
        {
            target = &targets_[i];
            highestPriority = targets_[i].getPriority();
        }
    }
    return target;
}

void Context::resetPriority()
{
    for (int i = 0; i < targets_.size(); ++i)
    {
        targets_[i].setPriority(0.0);
    }
}

}

