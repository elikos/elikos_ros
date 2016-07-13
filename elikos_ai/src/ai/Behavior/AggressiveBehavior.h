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
    virtual ~AggressiveBehavior();

    virtual void generateCommands(AbstractArena* arena);
    virtual bool isStateCritical(AbstractArena* arena);

private:
    TargetRobot* currentTarget_;
    void generateInteractionCommands(AbstractArena* arena);
    TargetRobot* chooseTargetRobot();

};

}


#endif // AI_AGGRESSIVE_BEHAVIOR_H
