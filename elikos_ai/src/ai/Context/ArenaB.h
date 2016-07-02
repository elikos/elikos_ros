//
// Created by olivier on 01/07/16.
//

#ifndef AI_ARENA_B_H
#define AI_ARENA_B_H

#include "AbstractArena.h"

namespace ai
{

class ArenaB : public AbstractArena
{
public:
    ArenaB();
    virtual ~ArenaB();
    virtual void evaluateTargetOrientation(const TargetRobot& robot, TargetOrientationEvaluation& evaluation);
};

}

#endif // AI_ARENA_B_H
