//
// Created by olivier on 27/06/16.
//

#ifndef AI_QUADDISTANCE_H
#define AI_QUADDISTANCE_H

#include "AbstractConsideration.h"

namespace ai
{

class QuadDistance : public AbstractConsideration
{
public:
    QuadDistance() = default;
    virtual ~QuadDistance();
    virtual void evaluatePriority(std::vector<TargetRobot>& targets, const QuadRobot& quad);

};

}

#endif // AI_QUADDISTANCE_H
