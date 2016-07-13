#ifndef AI_FACADE_H
#define AI_FACADE_H

#include <memory>
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

    enum Consideration
    {
        TARGET_DESTINATION,
        QUAD_DISTANCE,
        CLUSTER_SIZE
    };

    static Agent* getInstance();
    static void freeInstance();

    void updateTargets(const elikos_ros::TargetRobotArray::ConstPtr& input);
    void updateQuadRobot(const tf::Pose& pose);
    void addConsideration(Consideration consideration);

    void behave();

private:
    static Agent* instance_;

    std::unique_ptr<AbstractBehavior> behaviors_[2];
    AbstractBehavior* currentBehavior_;
    PriorityEvaluationPipeline pipeline_;

    AbstractBehavior* resolveCurrentBehavior();
    Agent();
    ~Agent() = default;
};

}

#endif /// AI_FACADE_H
