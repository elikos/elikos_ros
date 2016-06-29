//
// Created by olivier on 27/06/16.
//
#include "AbstractConsideration.h"

#include "ConsiderationPipeline.h"

namespace ai
{

    ConsiderationPipeline::ConsiderationPipeline()
    {
        for (uint8_t i = 0; i < 10; ++i)
        {
            targets_.push_back({i, 0});
        }
    }

    void ConsiderationPipeline::addConsideration(std::unique_ptr<AbstractConsideration> consideration)
    {
        considerations_.push_back(std::move(consideration));
    }

    TargetRobot* ConsiderationPipeline::evaluateTargetSelection()
    {
        for (int i = 0; i < considerations_.size(); ++i)
        {
            //TODO: Threads could be used here.
            considerations_[i]->evaluatePriority(targets_, quad_);
        }
        return findHighestPriorityTarget();
    }

    TargetRobot* ConsiderationPipeline::findHighestPriorityTarget()
    {
        TargetRobot* target = &targets_[0];
        double highestPriority = targets_[0].getPriority();
        for ( int i = 1; i < targets_.size(); ++i)
        {
            if (targets_[i].getPriority() > highestPriority)
            {
                target = &targets_[i];
                highestPriority = targets_[i].getPriority();
            }
        }
        return target;
    }


}
