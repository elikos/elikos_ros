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
    AbstractBehavior(AbstractArena* arena);
    virtual ~AbstractBehavior() = 0;
    void behave();
    virtual void generateCommands() = 0;
    virtual int resolveCurrentStateLevel() = 0;

protected:
    CommandQueue q_;
    AbstractArena* arena_;

private:
    AbstractBehavior() = default;
};

inline AbstractBehavior::~AbstractBehavior() {}

}


#endif // AI_ABSTRACT_BEHAVIOR_H
