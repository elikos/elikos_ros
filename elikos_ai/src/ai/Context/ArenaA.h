//
// Created by olivier on 01/07/16.
//

#ifndef AI_ARENA_A_H
#define AI_ARENA_A_H

#include "AbstractArena.h"

namespace ai
{

class ArenaA : public AbstractArena
{
public:
    ArenaA();
    virtual ~ArenaA();
    virtual void evaluateTargetOrientation(const TargetRobot& robot, TargetOrientationEvaluation& evaluation);

};

}

#endif // AI_ARENA_A_H
