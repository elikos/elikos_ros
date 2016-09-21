//
// Created by olivier on 07/07/16.
//

#ifndef AI_PREVENTIVE_BEHAVIOR_H
#define AI_PREVENTIVE_BEHAVIOR_H

#include "AbstractBehavior.h"
#include "PriorityEvaluationManager.h"

namespace ai
{

class Configuration;
class PreventiveBehavior : public AbstractBehavior
{
public:
    static constexpr double MAX_ACCEPTABLE_PRIORITY { 0.90 };

    PreventiveBehavior(AbstractArena* arena, Configuration* config);
    virtual ~PreventiveBehavior();

    virtual void generateCommands();
    virtual int resolveCurrentStateLevelConcrete();

private:
    PriorityEvaluationManager priorityManager_;

    PreventiveBehavior() = default;
};

}

#endif // AI_PREVENTIVE_BEHAVIOR_H
