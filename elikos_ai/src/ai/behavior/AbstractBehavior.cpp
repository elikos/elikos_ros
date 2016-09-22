//
// Created by olivier on 07/07/16.
//

#include "MovementCommand.h"

#include "AbstractBehavior.h"
#include "AbstractArena.h"
#include "AbstractConsideration.h"

namespace ai
{
AbstractBehavior::AbstractBehavior(AbstractArena* arena)
{
}

void AbstractBehavior::setIsEnabled(bool isEnabled)
{
    isEnabled_ = isEnabled;
}

void AbstractBehavior::behave()
{
    if (!q_.empty()) {
        q_.front()->execute();
        if (q_.front()->isCommmandDone()) {
            q_.pop();
        }
    } else {
        generateCommands();
    }
    arena_->prepareUpdate();
}

int AbstractBehavior::resolveCurrentStateLevel()
{
    int currentStateLevel = 0;
    if (isEnabled_)
    {
        currentStateLevel = resolveCurrentStateLevelConcrete();
    }
    return currentStateLevel;
}


void AbstractBehavior::updateTargetsPriority()
{
    for (size_t i = 0; i < considerations_.size(); i++)
    {
        considerations_[i]->evaluatePriority(arena_);
    }
}

}


