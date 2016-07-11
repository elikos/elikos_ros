//
// Created by olivier on 01/07/16.
//

#ifndef AI_ABSTRACT_ARENA_H
#define AI_ABSTRACT_ARENA_H

#include <memory>
#include <vector>
#include "AbstractLine.h"
#include "TargetOrientationEvaluation.h"

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

    virtual void evaluateTargetOrientation(const TargetRobot& target) = 0;
    virtual void populateTargets(std::vector<TargetRobot> target) = 0;
    virtual int getNRotationsForOptimalDirection(const TargetRobot& target) const = 0;

protected:
    std::vector<std::unique_ptr<AbstractLine>> lines_;
};

}

#endif // AI_ABSTRACT_ARENA_H
