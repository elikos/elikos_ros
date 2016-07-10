#ifndef AI_CONSIDERATION_PIPELINE_H
#define AI_CONSIDERATION_PIPELINE_H

#include <vector>

#include <memory>
#include "Robot/RobotTypes.h"
#include "AbstractConsideration.h"
#include "AbstractArena.h"
#include "Context.h"

class TargetRobot;

namespace ai
{

class PriorityEvaluationPipeline
{
public:
    PriorityEvaluationPipeline() = default;
    //TODO: add set/get for arena and add an option in the cmd parser
    ~PriorityEvaluationPipeline() = default;

    void addConsideration(std::unique_ptr<AbstractConsideration>);
    inline AbstractArena* getArena() const;
    TargetRobot* evaluatePriority(Context& context);

private:
    std::vector<std::unique_ptr<AbstractConsideration>> considerations_;
    std::unique_ptr<AbstractArena> arena_;

};

inline AbstractArena* PriorityEvaluationPipeline::getArena() const
{
    return arena_.get();
}

}


#endif // AI_CONSIDERATION_PIPELINE_H
