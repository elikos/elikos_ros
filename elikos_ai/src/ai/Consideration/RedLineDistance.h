//
// Created by olivier on 27/06/16.
//

#ifndef AI_RED_LINE_DISTANCE_H
#define AI_RED_LINE_DISTANCE_H

#include "AbstractConsideration.h"

namespace ai
{

class RedLineDistance : public AbstractConsideration
{
public:
    RedLineDistance() = default;
    virtual ~RedLineDistance();
    virtual void evaluatePriority(std::vector<TargetRobot>& targets, const QuadRobot& quad);
private:
    double findDistanceToClosestRedLine(const TargetRobot& target);
};

}

#endif //AI_RED_LINE_DISTANCE_H
