//
// Created by olivier on 01/07/16.
//

#include <iterator>

#include "AbstractArena.h"

#include "Agent.h"

namespace ai
{

const tf::Point AbstractArena::TOP_RIGHT_CORNER{ 10.0, 10.0, 0.0 };
const tf::Point AbstractArena::TOP_LEFT_CORNER{ -10.0, 10.0, 0.0 };
const tf::Point AbstractArena::BOTTOM_LEFT_CORNER{ -10.0, -10.0, 0.0 };
const tf::Point AbstractArena::BOTTOM_RIGHT_CORNER{ 10.0, -10.0, 0.0 };

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
        if (targets_[i].getPriority() > maxPriority &&
           !targets_[i].getOrientationEvaluation()->isOutOfBound_ &&
            targets_[i].getNMissedUpdates() < 10)
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
    std::cout << n << std::endl;
    for (int i = 0; i < targets_.size(); ++i)
    {
        targets_[i].prepareForUpdate();
    }
}

TargetRobot* AbstractArena::updateTarget(const elikos_ros::TargetRobot& targetUpdate)
{
    TargetRobot* target = findMostLikelyUpdateCondidate(targetUpdate);
    if (target != nullptr) {
        target->updateFrom(targetUpdate);
    }
    return target;
}

TargetRobot* AbstractArena::findMostLikelyUpdateCondidate(const elikos_ros::TargetRobot& targetUpdate)
{
    std::unordered_map<int, TargetRobot*>::iterator it = targetsId_.find(targetUpdate.id);
    TargetRobot* candidate = nullptr;
    // If this id is being tracked, juste update that robot
    if (it != targetsId_.end()) {
        int test = it->second->getId();
        candidate = it->second;
    } else {
        int maxNMissedUpdates = 0;
        for (int i = 0; i < targets_.size(); ++i)
        {
            int nMissedUpdates = targets_[i].getNMissedUpdates();
            if ( nMissedUpdates > maxNMissedUpdates &&
                 !targets_[i].getOrientationEvaluation()->isOutOfBound_ &&
                  targets_[i].getNMissedUpdates() > 0)
            {
                maxNMissedUpdates = nMissedUpdates;
                candidate = &targets_[i];
            }
        }
        // Insert the new id and erase the last one.
        if (candidate != nullptr) {
            targetsId_.erase(candidate->getId());
            targetsId_.insert({ targetUpdate.id, candidate });
        }
    }
    return candidate;

}

void AbstractArena::evaluateOutOfBound(TargetRobot& target)
{
    const double MIN = -9.5;
    const double MAX = 9.5;
    double x = target.getPose().getOrigin().x();
    double y = target.getPose().getOrigin().y();
    target.getOrientationEvaluation()->isOutOfBound_ = !(MIN <= x && x <= MAX && MIN <= y && y <= MAX);
}

}
