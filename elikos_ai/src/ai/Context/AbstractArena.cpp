//
// Created by olivier on 01/07/16.
//

#include "AbstractArena.h"

namespace ai
{
const tf::Point AbstractArena::TOP_RIGHT_CORNER    {  10.0,  10.0, 0.0 };
const tf::Point AbstractArena::TOP_LEFT_CORNER     { -10.0,  10.0, 0.0 };
const tf::Point AbstractArena::BOTTOM_LEFT_CORNER  { -10.0, -10.0, 0.0 };
const tf::Point AbstractArena::BOTTOM_RIGHT_CORNER {  10.0, -10.0, 0.0 };

AbstractArena::~AbstractArena()
{
}

}
