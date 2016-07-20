//
// Created by olivier on 01/07/16.
//

#include "AbstractArena.h"

#include "Agent.h"

namespace ai
{

const tf::Point AbstractArena::TOP_RIGHT_CORNER{10.0, 10.0, 0.0};
const tf::Point AbstractArena::TOP_LEFT_CORNER{-10.0, 10.0, 0.0};
const tf::Point AbstractArena::BOTTOM_LEFT_CORNER{-10.0, -10.0, 0.0};
const tf::Point AbstractArena::BOTTOM_RIGHT_CORNER{10.0, -10.0, 0.0};

AbstractArena::AbstractArena()
{
    timer_.start();
    targets_.resize(10);
}

AbstractArena::~AbstractArena()
{
}

TargetRobot* AbstractArena::findHighestPriorityTarget()
{
    double maxPriority = 0.0;
    TargetRobot* highestPriorityTarget = nullptr;
    for (int i = 0; i < targets_.size(); ++i)
    {
        if (targets_[i].getPriority() > maxPriority)
        {
            maxPriority = targets_[i].getPriority();
            highestPriorityTarget = &targets_[i];
        }
    }
    return highestPriorityTarget;
}

void AbstractArena::prepareUpdate()
{
    size_t n = targets_.size();
    for (int i = 0; i < targets_.size(); ++i)
    {
        if (targets_[i].getOrientationEvaluation()->isOutOfBound_) {
            targets_[i] = targets_[n - 1];
            targets_.pop_back();
            // This is a workaround, since we just deleted a target, we want to make sure there are no invalid pointers.
            // TODO: Need a better way of doing this.
            Agent::getInstance()->forceCommandGeneration();
        }
        targets_[i].prepareForUpdate();
    }
}

TargetRobot* AbstractArena::updateTarget(const elikos_ros::TargetRobot& targetUpdate)
{
    TargetRobot* target = findMostLikelyUpdateCondidate(targetUpdate);
    target->updateFrom(targetUpdate);
    return target;
}

TargetRobot* AbstractArena::findMostLikelyUpdateCondidate(const elikos_ros::TargetRobot& targetUpdate)
{
    std::unordered_map<int, TargetRobot*>::iterator it = targetsId_.find(targetUpdate.id);
    TargetRobot* candidate = nullptr;
    // If this id is being tracked, juste update that robot
    if (it != targetsId_.end()) {
        candidate = it->second;
    } else {
        // This is a new id, so find the most likely candidate
        double minDistance = 40.0;
        for (int i = 0; i < targets_.size(); ++i)
        {
            double distance = targets_[i].getDistance(targetUpdate);
            // We don't want to update a target discovered in this update iteration (its missed updates will be 0)
            if (distance < minDistance && targets_[i].getNMissedUpdates() != 0)
            {
                minDistance = distance;
                candidate = &targets_[i];
            }
        }
        // Insert the new id in the hashed table
        targetsId_.insert({ targetUpdate.id, candidate });
        // TODO: Maybe remove the least likely id.
    }
    return candidate;

}

void AbstractArena::evaluateOutOfBound(TargetRobot& target)
{
    const double MIN = -10.0;
    const double MAX = 10.0;
    double x = target.getPose().getOrigin().x();
    double y = target.getPose().getOrigin().y();

    if (!(MIN < x && x < MAX && MIN < y && y < MAX)) {
        target.getOrientationEvaluation()->isOutOfBound_ = true;
    }
}

}
