//
// Created by olivier on 27/06/16.
//
#include "AbstractConsideration.h"
#include "PriorityEvaluationPipeline.h"

#include "ArenaA.h"
#include "ArenaB.h"

namespace ai
{

void PriorityEvaluationPipeline::addConsideration(std::unique_ptr<AbstractConsideration> consideration)
{
    considerations_.push_back(std::move(consideration));
}


TargetRobot* PriorityEvaluationPipeline::evaluatePriority(Context& context)
{
    //TODO: evaluate after every update on a single robot so multithreading can be used.
    context.resetPriority();
    for (int i = 0; i < considerations_.size(); ++i)
    {
        considerations_[i]->evaluatePriority(context.getTargets()[i]);
    }
    return context.findHighestPriorityTarget();
}

}
