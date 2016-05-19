#ifndef AI_FOLLOWTARGET_H
#define AI_FOLLOWTARGET_H

#include "TargetSelectionStrategy.h"

class Robot;

namespace ai
{

class FollowClosestTarget : public TargetSelectionStrategy
{
public:
    FollowClosestTarget(QuadRobot& quad);
    virtual ~FollowClosestTarget();

    virtual Robot* findTargetSelection();
};

};

#endif /// AI_FOLLOWTARGET_H

