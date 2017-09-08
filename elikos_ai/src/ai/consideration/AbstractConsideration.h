//
// Created by olivier on 27/06/16.
//

#ifndef AI_ABSTRACT_CONSIDERATION_H
#define AI_ABSTRACT_CONSIDERATION_H

#include <vector>
#include "AbstractLine.h"
#include "TargetRobot.h"

namespace ai
{

class AbstractArena;
class QuadRobot;

class AbstractConsideration
{
public:
    AbstractConsideration(double weight);

    virtual ~AbstractConsideration() = 0;
    virtual void evaluatePriority(AbstractArena* arena) = 0;

private:
    double weight_ { 0.0 };

    AbstractConsideration() = default;
};

}

#endif //AI_ABSTRACT_CONSIDERATION_H
