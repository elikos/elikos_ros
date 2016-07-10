//
// Created by olivier on 07/07/16.
//

#ifndef AI_ABSTRACT_BEHAVIOR_H
#define AI_ABSTRACT_BEHAVIOR_H

#include "Context.h"
#include "CommandQueue.h"

namespace ai
{

class AbstractBehavior
{
public:
    AbstractBehavior() = default;
    virtual ~AbstractBehavior() = 0;
    virtual void generateCommands(CommandQueue& q, Context& context) = 0;
    virtual bool isContextCritical(Context& contest) = 0;

};

inline AbstractBehavior::~AbstractBehavior() {}

}


#endif // AI_ABSTRACT_BEHAVIOR_H
