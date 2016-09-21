//
// Created by olivier on 9/4/16.
//

#ifndef AI_BEHAVIOR_MANAGER_H
#define AI_BEHAVIOR_MANAGER_H

#include "BehaviorTypes.h"
#include "AbstractArena.h"

namespace ai
{

class AbstractArena;
class Configuration;

class BehaviorManager
{
public:
    BehaviorManager(AbstractArena* arena, Configuration* config);
    ~BehaviorManager();

    void behave();
    AbstractBehavior* resolveCurrentBehavior();

private:
    AggressiveBehavior aggressiveBehavior_;
    PreventiveBehavior preventiveBehavior_;
    ResearchBehavior researchBehavior_;

    BehaviorManager() = default;
};

}

#endif // AI_BEHAVIOR_MANAGER_H
