#ifndef ABSTRACT_STATE_H
#define ABSTRACT_STATE_H

#include <memory>

namespace ai
{

class StateMachine;

class AbstractState
{
public:

    AbstractState(StateMachine* reference);
    virtual ~AbstractState() = 0;
    virtual void handleTargetSelection(TargetRobot* target, const QuadRobot& quad) = 0;

protected:
    bool hasReachedDestination(const tf::Vector3& currentPosition, const tf::Vector3& destination);
    StateMachine* stateMachine_{ nullptr };
    // Deleted because a state needs a reference to the state machine

private:
    AbstractState() = delete;


};

}

#endif /// ABSTRACT_STATE_H
