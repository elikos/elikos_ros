#ifndef AI_FACADE_H
#define AI_FACADE_H

#include <memory>
#include <tf/tf.h>
#include "Context.h"
#include "CommandQueue.h"
#include "Robot/QuadRobot.h"
#include "PriorityEvaluationPipeline.h"
#include "AbstractBehavior.h"
#include <elikos_ros/TargetRobotArray.h>

namespace ai {

class Agent
{
public:
    enum EnumBehavior
    {
        PREVENTIVE,
        AGGRESSIVE
    };

    static Agent* getInstance();
    static void freeInstance();

    inline PriorityEvaluationPipeline* getConsiderationPipeline();

    inline void updateQuadRobot(const tf::Pose& pose);
    void updateTargets(const elikos_ros::TargetRobotArray::ConstPtr& input);
    void behave();

private:
    static Agent* instance_;
    Context context_;

    CommandQueue q_;
    std::unique_ptr<AbstractBehavior> behaviors_[2];
    AbstractBehavior* currentBehavior_;
    PriorityEvaluationPipeline pipeline_;

    AbstractBehavior* resolveCurrentBehavior();
    Agent();
    ~Agent() = default;
};

inline PriorityEvaluationPipeline* Agent::getConsiderationPipeline()
{
    return &pipeline_;
}

inline void Agent::updateQuadRobot(const tf::Pose& pose)
{
    context_.getQuad().setPose(pose);
}

}

#endif /// AI_FACADE_H
