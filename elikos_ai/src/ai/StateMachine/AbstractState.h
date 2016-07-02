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

    inline const tf::Point& getDestination() const;
    inline void setDestination(const tf::Point& destination);

protected:
    bool hasReachedDestination(const tf::Vector3& currentPosition, const tf::Vector3& destination);
    StateMachine* stateMachine_{ nullptr };
    tf::Point destination_;

private:
    // Deleted because a state needs a reference to the state machine
    AbstractState() = delete;
};

inline const tf::Point& AbstractState::getDestination() const
{
    return destination_;
}

inline void AbstractState::setDestination(const tf::Point& destination)
{
    destination_ = destination;
}

}

#endif /// ABSTRACT_STATE_H
