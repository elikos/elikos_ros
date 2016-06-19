#include "Strategy.h"
#include "RobotTypes.h"

namespace ai
{

Strategy::Strategy(QuadRobot& quad)
    : quad_(quad)
{
    for (int i = 0; i < N_TARGETS; i++)
    {
        targets_.push_back(TargetRobot(i, 0));
    }
}

Strategy::~Strategy()
{
}

};

