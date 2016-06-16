#ifndef AI_FOLLOWTARGET_H
#define AI_FOLLOWTARGET_H

#include "Strategy.h"

class Robot;

namespace ai
{

class FollowClosestTarget : public Strategy
{
public:
    FollowClosestTarget(QuadRobot& quad);
    virtual ~FollowClosestTarget();

    virtual Robot* findTargetSelection();
};

};

#endif /// AI_FOLLOWTARGET_H

