//
// Created by olivier on 01/07/16.
//

#include <iterator>

#include "AbstractArena.h"
#include "Agent.h"

namespace ai
{

AbstractArena::AbstractArena()
{
    timer_.start();
    targets_.resize(10);

    ros::NodeHandle nh;
    double dimension_c;
    nh.getParam("/elikos_ai/dimension_c", dimension_c);
    double edge_tolerance;
    nh.getParam("/elikos_ai/edge_tolerance", edge_tolerance);

    TOP_RIGHT_CORNER = tf::Point( dimension_c/2,  dimension_c/2, 0.0 );
    TOP_LEFT_CORNER = tf::Point(-dimension_c/2,  dimension_c/2, 0.0 );
    BOTTOM_LEFT_CORNER = tf::Point(-dimension_c/2, -dimension_c/2, 0.0 );
    BOTTOM_RIGHT_CORNER = tf::Point( dimension_c/2, -dimension_c/2, 0.0 );
    EDGE_TOLERANCE = edge_tolerance;
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
    for (int i = 0; i < targets_.size(); ++i)
    {
        targets_[i].prepareForUpdate();
    }
}

void AbstractArena::updateTargets(const elikos_ros::TargetRobotArray::ConstPtr& input)
{
    const elikos_ros::TargetRobot* inputTargets = input->targets.data();
    size_t n = input->targets.size();

    prepareUpdate();
    for (size_t i = 0; i < n; ++i)
    {
        updateTarget(inputTargets[i]);
    }
}

TargetRobot* AbstractArena::updateTarget(const elikos_ros::TargetRobot& targetUpdate)
{
    tf::Pose pose;
    tf::poseMsgToTF(targetUpdate.poseOrigin.pose, pose);

    TargetRobot* target = nullptr;
    if (!isOutOfBound(pose.getOrigin())) {
        target = findMostLikelyUpdateCondidate(targetUpdate);
        if (target != nullptr) {
            target->updateFrom(targetUpdate);
        }
    }
    return target;
}

void AbstractArena::updateQuadRobot(const tf::Pose& pose)
{
    quad_.setPose(pose);
}

TargetRobot* AbstractArena::findMostLikelyUpdateCondidate(const elikos_ros::TargetRobot& targetUpdate)
{
    std::unordered_map<int, TargetRobot*>::iterator it = targetsId_.find(targetUpdate.id);
    TargetRobot* candidate = nullptr;
    // If this id is being tracked, juste update that robot
    if (it != targetsId_.end()) {
        candidate = it->second;
    } else {
        int maxNMissedUpdates = 0;
        for (int i = 0; i < targets_.size(); ++i)
        {
            int nMissedUpdates = targets_[i].getNMissedUpdates();
            if ( nMissedUpdates > maxNMissedUpdates)
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
    double x = target.getPose().getOrigin().x();
    double y = target.getPose().getOrigin().y();
    target.getOrientationEvaluation()->isOutOfBound_ = isOutOfBound(target);
}

bool AbstractArena::isOutOfBound(tf::Point position)
{
    return !(TOP_LEFT_CORNER.getX() + EDGE_TOLERANCE <= position.x() && position.x() <= TOP_RIGHT_CORNER.getX() - EDGE_TOLERANCE &&
             BOTTOM_RIGHT_CORNER.getY() + EDGE_TOLERANCE <= position.y() && position.y() <= TOP_RIGHT_CORNER.getY() - EDGE_TOLERANCE);
}

bool AbstractArena::isOutOfBound(TargetRobot& target)
{
   return isOutOfBound(target.getPose().getOrigin());
}

int AbstractArena::getNbrOfUpdatedTargets()
{
    int nTargets = 0;
    for (int i = 0; i < targets_.size(); ++i) {
        if (targets_[i].getNMissedUpdates() < 10) {
            nTargets++;
        }
    }
    return nTargets;
}

}
