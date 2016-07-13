//
// Created by olivier on 27/06/16.
//

#ifndef AI_ABSTRACT_CONSIDERATION_H
#define AI_ABSTRACT_CONSIDERATION_H

#include <vector>

namespace ai
{

class AbstractArena;
class TargetRobot;

class AbstractConsideration
{
public:
    AbstractConsideration() = default;
    virtual ~AbstractConsideration() = 0;
    virtual void evaluatePriority(TargetRobot& robot, AbstractArena* arena) = 0;
};

}

#endif //AI_ABSTRACT_CONSIDERATION_H
