//
// Created by olivier on 27/06/16.
//

#ifndef AI_RED_LINE_DISTANCE_H
#define AI_RED_LINE_DISTANCE_H

#include "AbstractConsideration.h"

namespace ai
{

class AbstractArena;

class DestinationConsideration : public AbstractConsideration
{
public:
    DestinationConsideration() = default;
    virtual ~DestinationConsideration();
    virtual void evaluatePriority(AbstractArena* arena);
};

}

#endif //AI_RED_LINE_DISTANCE_H
