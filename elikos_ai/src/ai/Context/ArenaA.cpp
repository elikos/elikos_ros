//
// Created by olivier on 01/07/16.
//

#include "ArenaA.h"

namespace ai
{

ArenaA::ArenaA()
{
    lines_.push_back(std::unique_ptr<GreenLine>(new GreenLine(TOP_LEFT_CORNER, TOP_RIGHT_CORNER)));
    lines_.push_back(std::unique_ptr<WhiteLine>(new WhiteLine(BOTTOM_LEFT_CORNER, TOP_LEFT_CORNER)));
    lines_.push_back(std::unique_ptr<WhiteLine>(new WhiteLine(BOTTOM_RIGHT_CORNER, BOTTOM_LEFT_CORNER)));
    lines_.push_back(std::unique_ptr<WhiteLine>(new WhiteLine(TOP_RIGHT_CORNER, BOTTOM_RIGHT_CORNER)));
    targets_.resize(10);
}

ArenaA::~ArenaA()
{
}

void ArenaA::evaluateTargetOrientation(TargetRobot& robot)
{
    bool lineFound = false;
    for (int i = 0; i < lines_.size() && !lineFound; i++)
    {
        if (lines_[i]->isInThePath(robot))
        {
            lines_[i]->evaluate(robot);
            lineFound = true;
        }
    }
}

int ArenaA::getNRotationsForOptimalDirection(const TargetRobot& target) const
{
    tf::Vector3 direction = target.getDirection();
    tf::Vector3 possibleDirections[4]  {
            tf::Vector3( direction.x(),  direction.y(), direction.z()),
            tf::Vector3(-direction.y(),  direction.x(), direction.z()),
            tf::Vector3(-direction.x(), -direction.y(), direction.z()),
            tf::Vector3( direction.y(), -direction.x(), direction.z())
    };

    TargetRobot targetCpy(target);
    int nRobot = 0;
    bool lineFound = false;
    for (int i = 0; i < 4 && !lineFound; ++i)
    {
        nRobot = i;
        targetCpy.setDirection(possibleDirections[i]);
        // lines_[0] is the green line
        lineFound = lines_[0]->isInThePath(target);
    }
    return nRobot;
}

TargetRobot* ArenaA::findClosestTargetToGoodLine()
{
    double linePosition = 10.0;
    double minDistance = std::abs(targets_[0].getPose().getOrigin().y() - linePosition);
    TargetRobot* closestRobot = &targets_[0];
    for (int i = 1; i < targets_.size(); ++i)
    {
        double distance = std::abs(targets_[0].getPose().getOrigin().y() - linePosition);
        if (distance < minDistance)
        {
            minDistance = distance;
            closestRobot = &targets_[i];
        }
    }
    return closestRobot;
}

}
