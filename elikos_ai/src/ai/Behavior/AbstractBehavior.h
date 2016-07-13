//
// Created by olivier on 07/07/16.
//

#ifndef AI_ABSTRACT_BEHAVIOR_H
#define AI_ABSTRACT_BEHAVIOR_H

#include "CommandQueue.h"

namespace ai
{

class AbstractArena;

class AbstractBehavior
{
public:
    AbstractBehavior() = default;
    virtual ~AbstractBehavior() = 0;
    void behave();
    virtual void generateCommands(AbstractArena* arena) = 0;
    virtual bool isStateCritical(AbstractArena* arena) = 0;

protected:
    CommandQueue q_;
};

inline AbstractBehavior::~AbstractBehavior() {}

}


#endif // AI_ABSTRACT_BEHAVIOR_H
