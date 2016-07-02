#ifndef AI_FACADE_H
#define AI_FACADE_H

#include <memory>
#include <tf/tf.h>
#include "StateMachine.h"
#include "Robot/QuadRobot.h"
#include "ConsiderationPipeline.h"

namespace ai {

class Agent
{
public:
    static Agent* getInstance();
    static void freeInstance();

    inline ConsiderationPipeline* getConsiderationPipeline();
    inline void updateQuadRobot(const tf::Pose& pose);
    void behave();

private:
    static Agent* instance_;

    QuadRobot quad_;
    StateMachine stateMachine_;
    ConsiderationPipeline pipeline_;

    Agent();
    ~Agent() = default;
};

inline ConsiderationPipeline* Agent::getConsiderationPipeline()
{
    return &pipeline_;
}

inline void Agent::updateQuadRobot(const tf::Pose& pose)
{
    quad_.setPose(pose);
}

}

#endif /// AI_FACADE_H
