//
// Created by olivier on 9/4/16.
//

#include "BehaviorManager.h"
#include "Configuration.h"

namespace ai
{

BehaviorManager::BehaviorManager(AbstractArena* arena)
    : aggressiveBehavior_(arena), preventiveBehavior_(arena), researchBehavior_(arena)
{
    ros::NodeHandle nh;
    bool aggressive_enabled;
    nh.getParam("/elikos_ai/aggressive_enabled", aggressive_enabled);
    bool preventive_enabled;
    nh.getParam("/elikos_ai/preventive_enabled", preventive_enabled);
    bool research_enabled;
    nh.getParam("/elikos_ai/research_enabled", research_enabled);

    aggressiveBehavior_.setIsEnabled(aggressive_enabled);
    preventiveBehavior_.setIsEnabled(preventive_enabled);
    researchBehavior_.setIsEnabled(research_enabled);
}

BehaviorManager::~BehaviorManager()
{
}

void BehaviorManager::behave()
{
    AbstractBehavior* currentBehavior = resolveCurrentBehavior();
    if (currentBehavior != nullptr)
    {
        currentBehavior->behave();
    }
}

AbstractBehavior* BehaviorManager::resolveCurrentBehavior()
{
    AbstractBehavior* currentBehavior = nullptr;
    int researchStateLevel = researchBehavior_.resolveCurrentStateLevel();
    // Research behavior is either fine of disabled
    if (researchStateLevel < 2)
    {
        int preventiveStateLevel = preventiveBehavior_.resolveCurrentStateLevel();
        int aggressiveStateLevel = aggressiveBehavior_.resolveCurrentStateLevel();

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
