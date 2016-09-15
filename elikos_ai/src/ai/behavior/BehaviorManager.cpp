//
// Created by olivier on 9/4/16.
//

#include "BehaviorManager.h"
#include "Configuration.h"

namespace ai
{

BehaviorManager::BehaviorManager(AbstractArena* arena, Configuration* config)
    : arena_(arena)
{
    BehaviorConfig* behaviorConfig = config->getBehaviorConfig();
    aggressiveBehavior_.setIsEnabled(behaviorConfig->isAggressiveEnabled);
    preventiveBehavior_.setIsEnabled(behaviorConfig->isPreventiveEnabled);
    researchBehavior_.setIsEnabled(behaviorConfig->isResearchEnabled);
}

BehaviorManager::~BehaviorManager()
{
}

void BehaviorManager::behave()
{
    AbstractBehavior* currentBehavior = resolveCurrentBehavior();
    if (currentBehavior != nullptr)
    {
        currentBehavior->behave(arena_);
    }
}

AbstractBehavior* BehaviorManager::resolveCurrentBehavior()
{
    AbstractBehavior* currentBehavior = nullptr;
    int researchStateLevel = researchBehavior_.resolveCurrentStateLevel(arena_);
    // Research behavior is either fine of disabled
    if (researchStateLevel < 2)
    {
        int preventiveStateLevel = preventiveBehavior_.resolveCurrentStateLevel(arena_);
        int aggressiveStateLevel = aggressiveBehavior_.resolveCurrentStateLevel(arena_);

        if (preventiveStateLevel > aggressiveStateLevel && preventiveStateLevel > 0)
        {
            currentBehavior = &preventiveBehavior_;
        }
        else if (aggressiveStateLevel > 0)
        {
            currentBehavior = &aggressiveBehavior_;
        }
    }
    // researchStatelevel == 2: No targets detected for a while.
    else
    {
        currentBehavior = &researchBehavior_;
    }
    return currentBehavior;
}

}
