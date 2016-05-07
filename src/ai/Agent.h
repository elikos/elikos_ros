#ifndef AI_FACADE_H
#define AI_FACADE_H

#include <memory>
#include "StateMachine.h"

namespace ai
{

class Agent
{
public:
    Agent() = default;
    ~Agent() = default;

private:
    StateMachine stateMachine_;
};

}
#endif /// AI_FACADE_H
