#include "TargetSelectionStrategy.h"
#include "TargetRobot.h"

namespace ai
{

TargetSelectionStrategy::TargetSelectionStrategy(QuadRobot& quad)
    : quad_(quad)
{
    for (int i = 0; i < N_TARGETS; i++)
    {
        targets_.push_back(TargetRobot(i));
    }
}
TargetSelectionStrategy::~TargetSelectionStrategy()
{
}

};

