#ifndef AI_FOLLOWTARGET_H
#define AI_FOLLOWTARGET_H

#include "TargetSelectionStrategy.h"
namespace ai
{

class FollowClosestTarget : public TargetSelectionStrategy
{
public:
    FollowClosestTarget() = default;
    virtual ~FollowClosestTarget();

    virtual void updateTargetSelection();
};

};

#endif /// AI_FOLLOWTARGET_H

