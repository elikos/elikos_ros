//
// Created by olivier on 01/07/16.
//

#ifndef AI_ABSTRACT_ARENA_H
#define AI_ABSTRACT_ARENA_H

#include <memory>
#include <vector>
#include <unordered_map>

#include <elikos_main/TargetRobotArray.h>
#include "RobotTypes.h"
#include "Timer.h"

#include "AbstractLine.h"

namespace ai
{

class TargetRobot;
class AbstractConsideration;

class AbstractArena
{
public:
    double EDGE_TOLERANCE;

    tf::Point TOP_RIGHT_CORNER;
    tf::Point TOP_LEFT_CORNER;
    tf::Point BOTTOM_LEFT_CORNER;
    tf::Point BOTTOM_RIGHT_CORNER;

    AbstractArena();
    virtual ~AbstractArena() = 0;

    inline QuadRobot& getQuad();
    inline const std::unordered_map<int, TargetRobot*>& getTargets() const;
    inline const std::vector<AbstractLine*>& getLines() const;

    void prepareUpdate();
    void updateTargets(const elikos_main::TargetRobotArray::ConstPtr& input);
    void updateQuadRobot(const tf::Pose& pose);

    virtual void evaluateTargetOrientation(TargetRobot& target) = 0;
    virtual int getNRotationsForOptimalDirection(const TargetRobot& target) const = 0;
    virtual TargetRobot* findClosestTargetToGoodLine() = 0;

    bool isOutOfBound(tf::Point position);
    bool isOutOfBound(TargetRobot& target);

    TargetRobot* findHighestPriorityTarget();
    int getNbrOfUpdatedTargets();

protected:
    std::vector<AbstractLine*> lines_;
    std::vector<TargetRobot> targets_;
    std::unordered_map<int, TargetRobot*> targetsId_;

    QuadRobot quad_;
    util::Timer timer_;

    TargetRobot* updateTarget(const elikos_main::TargetRobot& targetUpdate);
    TargetRobot* findMostLikelyUpdateCondidate(const elikos_main::TargetRobot& targetUpdate);
    void evaluateOutOfBound(TargetRobot& target);
};

inline QuadRobot& AbstractArena::getQuad()
{
   return quad_;
}

inline const std::unordered_map<int, TargetRobot*>& AbstractArena::getTargets() const
{
    return targetsId_;
}

inline const std::vector<AbstractLine*>& AbstractArena::getLines() const
{
    return lines_;
}

}

#endif // AI_ABSTRACT_ARENA_H
