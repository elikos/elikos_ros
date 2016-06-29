//
// Created by olivier on 27/06/16.
//
#include <cmath>
#include <queue>
#include "RedLineDistance.h"
#include <vector>

namespace ai
{

RedLineDistance::~RedLineDistance()
{
}

void RedLineDistance::evaluatePriority(std::vector<TargetRobot>& targets, const QuadRobot& quad)
{
    for (int i = 0; i < targets.size(); ++i)
    {
        double distance = findDistanceToClosestRedLine(targets[i]);
        targets[i].setPriority((MAX_DISTANCE - distance) / MAX_DISTANCE);
    }

}

double RedLineDistance::findDistanceToClosestRedLine(const TargetRobot& target)
{
    const double x = target.getPose().getOrigin().getX();
    const double y = target.getPose().getOrigin().getY();

    std::priority_queue<double, std::vector<double>, std::greater<double>> distances;
    distances.push(std::abs(x - LEFT_SIDE));
    distances.push(std::abs(x - RIGHT_SIDE));
    distances.push(std::abs(y - DOWN_SIDE));
    return distances.top();
}


}
