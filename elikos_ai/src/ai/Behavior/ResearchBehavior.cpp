//
// Created by elikos on 29/07/16.
//

#include "ResearchBehavior.h"
#include "AbstractArena.h"
#include "CommandTypes.h"

namespace ai
{

ResearchBehavior::ResearchBehavior(bool isEnabled)
    : AbstractBehavior(isEnabled)
{
}

ResearchBehavior::~ResearchBehavior()
{
}

void ResearchBehavior::generateCommands(AbstractArena* arena)
{
    if (q_.empty()) {
    	q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena->getQuad(), {  0.0,  0.0, 2.0 })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena->getQuad(), {  0.0,  6.0, 2.0 })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena->getQuad(), { -6.0,  0.0, 2.0 })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena->getQuad(), {  6.0,  0.0, 2.0 })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena->getQuad(), {  0.0, -6.0, 2.0 })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena->getQuad(), {  0.0,  6.0, 2.0 })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena->getQuad(), {  6.0,  0.0, 2.0 })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena->getQuad(), { -6.0,  0.0, 2.0 })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena->getQuad(), {  0.0, -6.0, 2.0 })));
    }
}

int ResearchBehavior::resolveCurrentStateLevelConcrete(AbstractArena* arena)
{
    int stateLevel = 1;
    if (arena->getNbrOfUpdatedTargets() > 0) {
        stateLevel = 0;
    }
    return stateLevel;
}

}

