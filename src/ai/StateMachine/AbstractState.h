#ifndef ABSTRACT_STATE_H
#define ABSTRACT_STATE_H

#include <memory>

class AbstractState
{
public:
    AbstractState();
    ~AbstractState();

private:
    std::unique_ptr<AbstractState> nextState_;  
};

#endif /// ABSTRACT_STATE_H
