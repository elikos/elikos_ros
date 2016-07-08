//
// Created by olivier on 08/07/16.
//

#ifndef AI_CLUSTER_SIZE_H
#define AI_CLUSTER_SIZE_H

#include "AbstractConsideration.h"

namespace ai
{

class ClusterSize : public AbstractConsideration
{
public:
    ClusterSize() = default;
    virtual ~ClusterSize() = default;
    virtual void evaluatePriority(std::vector<TargetRobot>& targets, const QuadRobot& quad);
};

}

#endif // AI_CLUSTER_SIZE_H
