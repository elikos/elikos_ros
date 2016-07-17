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
    AggressiveBehavior(AbstractArena* arena);
    virtual ~AggressiveBehavior();

    virtual void generateCommands();
    virtual int resolveCurrentStateLevel();

private:
    TargetRobot* currentTarget_;
    AggressiveBehavior() = default;
};

}


#endif // AI_AGGRESSIVE_BEHAVIOR_H
