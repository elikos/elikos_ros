//
// Created by olivier on 27/06/16.
//

#ifndef AI_ABSTRACT_CONSIDERATION_H
#define AI_ABSTRACT_CONSIDERATION_H

#include "Robot/RobotTypes.h"
#include <vector>


namespace ai
{

class AbstractArena;

class AbstractConsideration
{
public:
    AbstractConsideration() = default;
    AbstractConsideration(AbstractArena* arena);
    virtual ~AbstractConsideration() = 0;
    virtual void evaluatePriority(std::vector<TargetRobot>& targets, const QuadRobot& quad) = 0;

protected:
    AbstractArena* arena_;
};

}

#endif //AI_ABSTRACT_CONSIDERATION_H
