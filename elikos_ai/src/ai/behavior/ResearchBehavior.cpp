//
// Created by elikos on 29/07/16.
//

#include "ResearchBehavior.h"
#include "AbstractArena.h"
#include "CommandTypes.h"

namespace ai
{

ResearchBehavior::ResearchBehavior(AbstractArena* arena)
    : AbstractBehavior(arena)
{
}

ResearchBehavior::~ResearchBehavior()
{
}

void ResearchBehavior::generateCommands()
{
    if (q_.empty()) {
    	q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena_->getQuad(), {  0.0,  0.0, 2.0 })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena_->getQuad(), {  0.0,  6.0, 2.0 })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena_->getQuad(), { -6.0,  0.0, 2.0 })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena_->getQuad(), {  6.0,  0.0, 2.0 })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena_->getQuad(), {  0.0, -6.0, 2.0 })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena_->getQuad(), {  0.0,  6.0, 2.0 })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena_->getQuad(), {  6.0,  0.0, 2.0 })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena_->getQuad(), { -6.0,  0.0, 2.0 })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena_->getQuad(), {  0.0, -6.0, 2.0 })));
    }
}

int ResearchBehavior::resolveCurrentStateLevelConcrete()
{
    int stateLevel = 2;
    if (arena_->getNbrOfUpdatedTargets() > 0) {
        stateLevel = 1;
    }
    return stateLevel;
}

}

