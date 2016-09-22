//
// Created by olivier on 01/07/16.
//

#include "ArenaA.h"

#include "Configuration.h"
#include <iterator>

namespace ai
{

ArenaA::ArenaA(Configuration* config)
    : greenLine_  { GreenLine(TOP_LEFT_CORNER,     TOP_RIGHT_CORNER)},
      whiteLines_ { WhiteLine(BOTTOM_LEFT_CORNER,  TOP_LEFT_CORNER),
                    WhiteLine(BOTTOM_RIGHT_CORNER, BOTTOM_LEFT_CORNER),
                    WhiteLine(TOP_RIGHT_CORNER,    BOTTOM_RIGHT_CORNER)}
{
    ArenaConfig* arenaConfig = config->getArenaConfig();
    lines_.push_back(&greenLine_);
    for (int i = 0; i < 3; i++)
    {
        lines_.push_back(&whiteLines_[i]);
    }
}

ArenaA::~ArenaA()
{
    for (int i = 0; i < lines_.size(); ++i)
    {
        delete lines_[i];
        lines_[i] = nullptr;
    }
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
        evaluateOutOfBound(robot);
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
    double minDistance = 20;
    TargetRobot* closestRobot = nullptr;
    for (int i = 0; i < targets_.size(); ++i)
    {
        double distance = std::abs(targets_[i].getPose().getOrigin().y() - linePosition);
        if (distance < minDistance &&
            targets_[i].getNMissedUpdates() < 10)
        {
            minDistance = distance;
            closestRobot = &targets_[i];
        }
    }
    return closestRobot;
}

}
