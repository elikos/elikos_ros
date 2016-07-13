#ifndef AI_CONSIDERATION_PIPELINE_H
#define AI_CONSIDERATION_PIPELINE_H

#include <vector>

#include <memory>
#include "AbstractArena.h"
#include "AbstractConsideration.h"

class TargetRobot;

namespace ai
{

class PriorityEvaluationPipeline
{
public:
    PriorityEvaluationPipeline();
    ~PriorityEvaluationPipeline() = default;

    //TODO: add set/get for arena and add an option in the cmd parser
    void addConsideration(std::unique_ptr<AbstractConsideration>);

    inline void updateQuadRobot(const tf::Pose& pose);
    inline AbstractArena* getArena();
    void updateTargets(const elikos_ros::TargetRobotArray::ConstPtr& input);
    TargetRobot* findHighestPriorityTarget();

private:
    std::vector<std::unique_ptr<AbstractConsideration>> considerations_;
    std::unique_ptr<AbstractArena> arena_;

    void updateTarget(const elikos_ros::TargetRobot& target, int i);
    void evaluatePriority(TargetRobot& target);

};

void PriorityEvaluationPipeline::updateQuadRobot(const tf::Pose& pose)
{
    arena_->getQuad().setPose(pose);
}

inline AbstractArena* PriorityEvaluationPipeline::getArena()
{
    return arena_.get();
}

}


#endif // AI_CONSIDERATION_PIPELINE_H
