#ifndef ABSTRACT_STATE_H
#define ABSTRACT_STATE_H

#include <memory>

class State
{
public:
    State();
    ~State();

private:
    std::unique_ptr<State> nextState_;
};

#endif /// ABSTRACT_STATE_H
