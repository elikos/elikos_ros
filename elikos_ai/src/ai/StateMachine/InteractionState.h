//
// Created by olivier on 28/06/16.
//

#ifndef ELIKOS_AI_INTERACTIONSTATE_H
#define ELIKOS_AI_INTERACTIONSTATE_H


#include "AbstractState.h"

namespace ai
{

class InteractionState : public AbstractState
{
    InteractionState() = default;
    virtual ~InteractionState();
};

}

#endif //ELIKOS_AI_INTERACTIONSTATE_H

