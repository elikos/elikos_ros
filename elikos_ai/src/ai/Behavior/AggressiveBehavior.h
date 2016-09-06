//
// Created by olivier on 07/07/16.
//

#ifndef AI_AGGRESSIVE_BEHAVIOR_H
#define AI_AGGRESSIVE_BEHAVIOR_H

#include "AbstractBehavior.h"
#include "TargetDestination.h"

namespace ai
{

class AggressiveBehavior : public AbstractBehavior
{
public:
    AggressiveBehavior() = default;
    AggressiveBehavior(bool isEnabled);
    virtual ~AggressiveBehavior();

    virtual void generateCommands(AbstractArena* arena);
    virtual int resolveCurrentStateLevelConcrete(AbstractArena* arena);

private:
    TargetRobot* currentTarget_;
};

}


#endif // AI_AGGRESSIVE_BEHAVIOR_H
