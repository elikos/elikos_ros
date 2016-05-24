#ifndef AI_FACADE_H
#define AI_FACADE_H

#include <memory>
#include <tf/tf.h>
#include "StateMachine.h"
#include "StrategyTypes.h"
#include "QuadRobot.h"

namespace ai {

class Agent
{
public:

    static Agent* getInstance();
    static void freeInstance();

    inline void updateTarget(const int& id, const tf::Vector3& position, const tf::Quaternion& orientation);
    inline void updateObstacle(const int& id, const tf::Vector3& position, const tf::Quaternion& orientation);
    inline void updateQuadRobot(const tf::Vector3& position, const tf::Quaternion& orientation);

    void behave();

private:
    static Agent* instance_;

    QuadRobot quad_;
    StateMachine stateMachine_;
    std::unique_ptr<Strategy> strategy_;

    Agent();
    ~Agent() = default;

};

inline void Agent::updateTarget(const int& id, const tf::Vector3& position, const tf::Quaternion& orientation)
{
    strategy_->updateTarget(id, position, orientation);
}

inline void Agent::updateObstacle(const int& id, const tf::Vector3& position, const tf::Quaternion& orientation)
{
    //TODO: Ask the safety evaluation if the situation is safe.
}

inline void Agent::updateQuadRobot(const tf::Vector3& position, const tf::Quaternion& orientation)
{
    quad_.setPosition(position);
    quad_.setOrientation(orientation);
}

}; /// namespace ai

#endif /// AI_FACADE_H
