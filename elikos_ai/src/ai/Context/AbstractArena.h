//
// Created by olivier on 01/07/16.
//

#ifndef AI_ABSTRACT_ARENA_H
#define AI_ABSTRACT_ARENA_H

#include <memory>
#include <vector>

#include "RobotTypes.h"
#include "AbstractLine.h"
#include <elikos_ros/TargetRobotArray.h>

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

    AbstractArena() = default;
    virtual ~AbstractArena() = 0;

    inline QuadRobot& getQuad();
    TargetRobot* updateTarget(const elikos_ros::TargetRobot& targetUpdate, int i);

    virtual void evaluateTargetOrientation(TargetRobot& target) = 0;
    virtual int getNRotationsForOptimalDirection(const TargetRobot& target) const = 0;
    virtual TargetRobot* findClosestTargetToGoodLine() = 0;

    TargetRobot* findHighestPriorityTarget();
    void resetPriority();

protected:
    std::vector<std::unique_ptr<AbstractLine>> lines_;
    std::vector<TargetRobot> targets_;
    QuadRobot quad_;
};

inline QuadRobot& AbstractArena::getQuad()
{
   return quad_;
}


}

#endif // AI_ABSTRACT_ARENA_H
