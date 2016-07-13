//
// Created by olivier on 07/07/16.
//

#ifndef AI_PREVENTIVE_BEHAVIOR_H
#define AI_PREVENTIVE_BEHAVIOR_H

#include "AbstractBehavior.h"

namespace ai
{

class PreventiveBehavior : public AbstractBehavior
{
public:
    static constexpr double MIN_ACCEPTABLE_LINE_DISTANCE { 1.0 };

    PreventiveBehavior() = default;
    virtual ~PreventiveBehavior();

    virtual void generateCommands(AbstractArena* arena);
    virtual bool isStateCritical(AbstractArena* arena);

private:
    bool isAlreadyCritical = false;

};

}

#endif // AI_PREVENTIVE_BEHAVIOR_H
