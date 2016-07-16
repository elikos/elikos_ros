//
// Created by olivier on 07/07/16.
//

#include "AbstractBehavior.h"

namespace ai
{
AbstractBehavior::AbstractBehavior(AbstractArena* arena)
    : arena_(arena)
{
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
}

}
