#ifndef AI_FACADE_H
#define AI_FACADE_H

#include <memory>
#include <tf/tf.h>
#include "StateMachine.h"
#include "StrategyTypes.h"

namespace ai
{

class Agent
{
public:
    Agent();
    ~Agent() = default;

    void updateTarget(const int& id, const tf::Vector3& position, const tf::Quaternion& orientation);
    void updateObstacle(const int& id, const tf::Vector3& position, const tf::Quaternion& orientation);
    void updateMAV(const tf::Vector3& position, const tf::Quaternion& orientation);

    void takeADecision();


private:
    StateMachine stateMachine_;
    std::unique_ptr<TargetSelectionStrategy> strategy_;
};

}
#endif /// AI_FACADE_H
