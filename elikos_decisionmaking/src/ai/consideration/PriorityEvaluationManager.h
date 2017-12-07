#ifndef AI_PRIORITY_EVALUATION_MANAGER_H
#define AI_PRIORITY_EVALUATION_MANAGER_H

#include <vector>
#include "AbstractConsideration.h"
#include <elikos_msgs/TargetRobotArray.h>
#include "tf/tf.h"

namespace ai
{

class TargetRobot;
class AbstractArena;
class Configuration;

class PriorityEvaluationManager
{
public:
    PriorityEvaluationManager(AbstractArena* arena);
    ~PriorityEvaluationManager() = default;

    void updateQuadRobot(const tf::Pose& pose);
    void updateTargets(const elikos_msgs::TargetRobotArray::ConstPtr& input);

private:
    std::vector<std::unique_ptr<AbstractConsideration>> considerations_;
    AbstractArena* arena_;

    void updateTarget(const elikos_msgs::TargetRobot& target);
    void evaluatePriority(TargetRobot& target);

    PriorityEvaluationManager() = default;

};

}

#endif // AI_PRIORITY_EVALUATION_PIPELINE_H
