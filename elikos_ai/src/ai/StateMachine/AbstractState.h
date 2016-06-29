#ifndef ABSTRACT_STATE_H
#define ABSTRACT_STATE_H

#include <memory>
namespace ai
{

class AbstractState
{
public:
    AbstractState() = default;
    virtual ~AbstractState() = 0;
};

}

#endif /// ABSTRACT_STATE_H
