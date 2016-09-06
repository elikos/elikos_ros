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
    AbstractBehavior(bool isEnabled);
    virtual ~AbstractBehavior() = 0;

    void setIsEnabled(bool isEnabled);

    void behave(AbstractArena* arena);
    virtual void generateCommands(AbstractArena* arena) = 0;
    int resolveCurrentStateLevel(AbstractArena* arena);

protected:
    CommandQueue q_;
    virtual int resolveCurrentStateLevelConcrete(AbstractArena* arena) = 0;

private:
    bool isEnabled_{ true };
};

inline AbstractBehavior::~AbstractBehavior() {}

}

#endif // AI_ABSTRACT_BEHAVIOR_H
