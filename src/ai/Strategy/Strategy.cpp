#include "Strategy.h"
#include "RobotTypes.h"

namespace ai
{

TargetSelectionStrategy::Strategy(QuadRobot& quad)
    : quad_(quad)
{
    for (int i = 0; i < N_TARGETS; i++)
    {
        targets_.push_back(TargetRobot(i));
    }
}
TargetSelectionStrategy::~Strategy()
{
}

};

