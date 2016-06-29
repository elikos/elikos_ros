//
// Created by olivier on 28/06/16.
//

#ifndef ELIKOS_AI_MOVING_H
#define ELIKOS_AI_MOVING_H

#include "AbstractState.h"

namespace ai
{

class MovementState : AbstractState
{
public:
    MovementState() = default;
    virtual ~MovementState();
};

}

#endif //ELIKOS_AI_MOVING_H
