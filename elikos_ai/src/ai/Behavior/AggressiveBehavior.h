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

    virtual void generateCommands(Context& context);
    virtual bool isContextCritical(Context& context);

private:
    TargetRobot* currentTarget_;
    void generateInteractionCommands(Context& context);
    TargetRobot* chooseTargetRobot();
};

}


#endif // AI_AGGRESSIVE_BEHAVIOR_H
