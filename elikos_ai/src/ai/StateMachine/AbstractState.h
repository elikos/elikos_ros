#ifndef ABSTRACT_STATE_H
#define ABSTRACT_STATE_H

#include <memory>

namespace ai
{

class StateMachine;
class TargetRobot;
class QuadRobot;

class AbstractState
{
public:
    AbstractState(StateMachine* stateMachine, QuadRobot* quad);
    virtual ~AbstractState() = 0;

    inline void setTarget(TargetRobot* target);

    virtual void handlePriorityUpdate(TargetRobot* highestPriority);
    virtual void behave() = 0;

protected:
    bool hasReachedDestination(const tf::Vector3& currentPosition, const tf::Vector3& destination);
    StateMachine* stateMachine_;
    QuadRobot* quad_;
    TargetRobot* target_;
    //TODO: use command pattern with a q for handling more complexe trajectories.

private:
    // Deleted because a state needs a reference to the state machine
    AbstractState() = delete;
};

inline void AbstractState::setTarget(TargetRobot* target)
{
    target_ = target;
}

}

#endif /// ABSTRACT_STATE_H
