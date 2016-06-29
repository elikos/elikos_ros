#ifndef AI_FACADE_H
#define AI_FACADE_H

#include <memory>
#include <tf/tf.h>
#include "StateMachine.h"
#include "QuadRobot.h"
#include "ConsiderationPipeline.h"

namespace ai {

class Agent
{
public:
    static Agent* getInstance();
    static void freeInstance();

    inline void updateTarget(const uint8_t& id, const uint8_t& color, const tf::Pose pose);
    inline void updateQuadRobot(const tf::Pose& pose);

    inline ConsiderationPipeline* getConsiderationPipeline();

    void behave();

private:
    static Agent* instance_;

    QuadRobot quad_;
    StateMachine stateMachine_;
    ConsiderationPipeline pipeline_;

    Agent();
    ~Agent() = default;
};

inline void Agent::updateTarget(const uint8_t& id, const uint8_t& color, const tf::Pose pose)
{
    pipeline_.updateTarget(id, color, pose);
}

inline void Agent::updateQuadRobot(const tf::Pose& pose)
{
    quad_.setPose(pose);
}

inline ConsiderationPipeline* Agent::getConsiderationPipeline()
{
    return &pipeline_;
}

}; /// namespace ai

#endif /// AI_FACADE_H
