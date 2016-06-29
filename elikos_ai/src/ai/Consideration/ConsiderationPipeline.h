#ifndef AI_CONSIDERATION_PIPELINE_H
#define AI_CONSIDERATION_PIPELINE_H

#include <vector>
#include <memory>
#include "RobotTypes.h"
#include "AbstractConsideration.h"

class TargetRobot;

namespace ai
{

class ConsiderationPipeline
{
public:
    ConsiderationPipeline();
    ~ConsiderationPipeline() = default;

    void addConsideration(std::unique_ptr<AbstractConsideration>);
    inline void updateTarget(const uint8_t& id, const uint8_t& color, const tf::Pose& pose);
    TargetRobot* evaluateTargetSelection();

private:
    std::vector<std::unique_ptr<AbstractConsideration>> considerations_;
    std::vector<TargetRobot> targets_;
    QuadRobot quad_;

    TargetRobot* findHighestPriorityTarget();
};

inline void ConsiderationPipeline::updateTarget(const uint8_t& id, const uint8_t& color, const tf::Pose& pose)
{
    targets_[id].setPose(pose);
    targets_[id].setColor(color);
    targets_[id].setIsUpdated(true);
}

}


#endif // AI_CONSIDERATION_PIPELINE_H
