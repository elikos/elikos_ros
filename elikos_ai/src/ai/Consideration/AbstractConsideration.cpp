//
// Created by olivier on 27/06/16.
//

#include "AbstractConsideration.h"

namespace ai
{
const tf::Vector3 AbstractConsideration::corners[N_CORNERS] = {{  10.0,  10.0, 0.0 },
                                                               { -10.0,  10.0, 0.0 },
                                                               { -10.0, -10.0, 0.0 },
                                                               {  10.0, -10.0, 0.0 }};

    AbstractConsideration::~AbstractConsideration()
    {
    }
}