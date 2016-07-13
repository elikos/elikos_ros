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
    inline std::vector<TargetRobot>& getTargets();

    virtual void evaluateTargetOrientation(TargetRobot& target) = 0;
    virtual int getNRotationsForOptimalDirection(const TargetRobot& target) const = 0;

    TargetRobot* findHighestPriorityTarget();
    void resetPriority();
    void updateTargets(const elikos_ros::TargetRobotArray::ConstPtr& input);

protected:
    std::vector<std::unique_ptr<AbstractLine>> lines_;
    std::vector<TargetRobot> targets_;
    QuadRobot quad_;
};

inline QuadRobot& AbstractArena::getQuad()
{
   return quad_;
}

inline std::vector<TargetRobot>& AbstractArena::getTargets()
{
   return targets_;
}

}

#endif // AI_ABSTRACT_ARENA_H
