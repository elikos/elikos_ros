//
// Created by olivier on 07/07/16.
//

#include "MovementCommand.h"

#include "AbstractBehavior.h"
#include "AbstractArena.h"

namespace ai
{
AbstractBehavior::AbstractBehavior(bool isEnabled)
    : isEnabled_(isEnabled)
{
}

void AbstractBehavior::setIsEnabled(bool isEnabled)
{
    isEnabled_ = isEnabled;
}

void AbstractBehavior::behave(AbstractArena* arena)
{
    if (!q_.empty()) {
        q_.front()->execute();
        if (q_.front()->isCommmandDone()) {
            q_.pop();
        }
    } else {
        generateCommands(arena);
    }
    arena->prepareUpdate();
}

int AbstractBehavior::resolveCurrentStateLevel(AbstractArena* arena)
{
    int currentStateLevel = 0;
    if (isEnabled_)
    {
        currentStateLevel = resolveCurrentStateLevelConcrete(arena);
    }
    return currentStateLevel;
}

}


