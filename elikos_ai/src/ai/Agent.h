#ifndef AI_FACADE_H
#define AI_FACADE_H

#include <memory>

#include "Timer.h"
#include "PriorityEvaluationManager.h"
#include "BehaviorManager.h"
#include "AbstractArena.h"
#include <elikos_main/TargetRobotArray.h>

namespace ai
{

class Agent
{
public:

    enum ConsiderationType
    {
        TARGET_DESTINATION,
        QUAD_DISTANCE,
        CLUSTER_SIZE
    };

    static Agent* getInstance();
    static void freeInstance();

    void init();

    void updateTargets(const elikos_main::TargetRobotArray::ConstPtr& input);
    void updateQuadRobot(const tf::Pose& pose);

    void behave();

private:
    static Agent* instance_;

    std::unique_ptr<BehaviorManager> behaviorManager_;
    std::unique_ptr<PriorityEvaluationManager> priorityManager_;
    std::unique_ptr<AbstractArena> arena_;

    Agent();
    ~Agent() = default;
};

}

#endif /// AI_FACADE_H
