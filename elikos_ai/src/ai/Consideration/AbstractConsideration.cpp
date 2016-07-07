//
// Created by olivier on 27/06/16.
//

#include "AbstractArena.h"

#include "AbstractConsideration.h"

namespace ai
{

AbstractConsideration::AbstractConsideration(AbstractArena *arena)
    : arena_(arena)
{
}

AbstractConsideration::~AbstractConsideration()
{
}

}