//
// Created by elikos on 29/07/16.
//

#ifndef AI_TARGET_RESEARCH_H
#define AI_TARGET_RESEARCH_H

#include "AbstractBehavior.h"

namespace ai
{

class AbstractArena;

class TargetResearch : public AbstractBehavior
{
public:
    TargetResearch(AbstractArena* arena);
    virtual ~TargetResearch();

    virtual void generateCommands();
    virtual int  resolveCurrentStateLevel();

private:
    TargetResearch() = default;
};

}

#endif //ELIKOS_AI_TARGETRESEARCH_H
