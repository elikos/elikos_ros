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
}

BehaviorManager::~BehaviorManager()
{
}

void BehaviorManager::behave()
{
    AbstractBehavior* currentBehavior = resolveCurrentBehavior();
    currentBehavior->behave(arena_);
}

AbstractBehavior* BehaviorManager::resolveCurrentBehavior()
{
    AbstractBehavior* currentBehavior = &researchBehavior_;

    int researchStateLevel = researchBehavior_.resolveCurrentStateLevel(arena_);
    if (researchStateLevel == 0) {
        int preventiveStateLevel = preventiveBehavior_.resolveCurrentStateLevel(arena_);
        int aggressiveStateLevel = aggressiveBehavior_.resolveCurrentStateLevel(arena_);

        if (preventiveStateLevel > aggressiveStateLevel) {
            currentBehavior = &preventiveBehavior_;
        } else {
            currentBehavior = &aggressiveBehavior_;
        }
    }

    return currentBehavior;
}

}
