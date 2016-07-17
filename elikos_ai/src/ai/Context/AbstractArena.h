//
// Created by olivier on 01/07/16.
//

#ifndef AI_ABSTRACT_ARENA_H
#define AI_ABSTRACT_ARENA_H

#include <memory>
#include <mutex>
#include <vector>
#include <map>

#include <elikos_ros/TargetRobotArray.h>
#include "RobotTypes.h"
#include "Timer.h"

#include "AbstractLine.h"



namespace ai
{

class TargetRobot;

class AbstractArena
{
public:
    static const tf::Point TOP_RIGHT_CORNER;
    static const tf::Point TOP_LEFT_CORNER;
    static const tf::Point BOTTOM_LEFT_CORNER;
    static const tf::Point BOTTOM_RIGHT_CORNER;

    AbstractArena();
    virtual ~AbstractArena() = 0;

    inline QuadRobot& getQuad();

    void prepareUpdate();
    TargetRobot* updateTarget(const elikos_ros::TargetRobot& targetUpdate);

    virtual void evaluateTargetOrientation(TargetRobot& target) = 0;
    virtual int getNRotationsForOptimalDirection(const TargetRobot& target) const = 0;
    virtual TargetRobot* findClosestTargetToGoodLine() = 0;

    TargetRobot* findHighestPriorityTarget();

protected:
    std::vector<std::unique_ptr<AbstractLine>> lines_;
    std::vector<TargetRobot> targets_;
    std::vector<int> noUpdateFlags_;
    std::mutex mutex_;

    QuadRobot quad_;
    util::Timer timer_;

    int findMostLikelyUpdateCondidate(const elikos_ros::TargetRobot& targetUpdate) const;
    void evaluateOutOfBound(TargetRobot& target);
};

inline QuadRobot& AbstractArena::getQuad()
{
   return quad_;
}


}

#endif // AI_ABSTRACT_ARENA_H
