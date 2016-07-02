#ifndef AI_CONSIDERATION_PIPELINE_H
#define AI_CONSIDERATION_PIPELINE_H

#include <vector>

#include <memory>
#include "Robot/RobotTypes.h"
#include "AbstractConsideration.h"
#include "AbstractArena.h"

class TargetRobot;

namespace ai
{

class PriorityEvaluationPipeline
{
public:
    PriorityEvaluationPipeline(QuadRobot& quad);
    //TODO: add set/get for arena and add an option in the cmd parser
    ~PriorityEvaluationPipeline() = default;

    void addConsideration(std::unique_ptr<AbstractConsideration>);
    inline void updateTarget(uint8_t id, uint8_t color, const tf::Pose& pose);
    inline AbstractArena* getArena() const;
    void resetPriority();
    TargetRobot* evaluateTargetSelection(const QuadRobot& quad);

private:
    std::vector<TargetRobot> targets_;
    QuadRobot& quad_;
    std::vector<std::unique_ptr<AbstractConsideration>> considerations_;
    std::unique_ptr<AbstractArena> arena_;

    TargetRobot* findHighestPriorityTarget();
    PriorityEvaluationPipeline() = delete;
};

inline void PriorityEvaluationPipeline::updateTarget(uint8_t id, uint8_t color, const tf::Pose& pose)
{
    targets_[id].setPose(pose);
    targets_[id].setColor(color);
    targets_[id].setIsUpdated(true);
}

inline AbstractArena* PriorityEvaluationPipeline::getArena() const
{
    return arena_.get();
}

}


#endif // AI_CONSIDERATION_PIPELINE_H
