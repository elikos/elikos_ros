//
// Created by elikos on 29/07/16.
//

#ifndef AI_TARGET_RESEARCH_H
#define AI_TARGET_RESEARCH_H

#include "AbstractBehavior.h"

namespace ai
{

class AbstractArena;

class ResearchBehavior : public AbstractBehavior
{
public:
    ResearchBehavior() = default;
    ResearchBehavior(AbstractArena* arena);
    virtual ~ResearchBehavior();

    virtual void generateCommands();
    virtual int  resolveCurrentStateLevelConcrete();
};

}

#endif // AI_TARGET_RESEARCH_H
