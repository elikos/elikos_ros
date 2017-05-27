//
// Created by olivier on 01/07/16.
//

#ifndef AI_ARENA_A_H
#define AI_ARENA_A_H

#include "AbstractArena.h"

namespace ai
{

class TargetRobot;
class Configuration;

class ArenaA : public AbstractArena
{
public:
    ArenaA();
    virtual ~ArenaA();

    virtual void evaluateTargetOrientation(TargetRobot& target);
    virtual int getNRotationsForOptimalDirection(const TargetRobot& target) const;

    virtual TargetRobot* findClosestTargetToGoodLine();

    WhiteLine* whiteLines_[3];
    GreenLine* greenLine_;

};

}

#endif // AI_ARENA_A_H
