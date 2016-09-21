//
// Created by olivier on 07/07/16.
//

#ifndef AI_ABSTRACT_BEHAVIOR_H
#define AI_ABSTRACT_BEHAVIOR_H

#include <context/AbstractArena.h>
#include "CommandQueue.h"

namespace ai
{

class AbstractArena;

class AbstractBehavior
{
public:
    AbstractBehavior(AbstractArena* arena);
    virtual ~AbstractBehavior() = 0;

    void setIsEnabled(bool isEnabled);

    void behave();
    virtual void generateCommands() = 0;
    int resolveCurrentStateLevel();

protected:
    CommandQueue q_;
    AbstractArena* arena_;
    virtual int resolveCurrentStateLevelConcrete() = 0;

private:
    bool isEnabled_{ true };
    AbstractBehavior() = default;
};

inline AbstractBehavior::~AbstractBehavior() {}

}

#endif // AI_ABSTRACT_BEHAVIOR_H
