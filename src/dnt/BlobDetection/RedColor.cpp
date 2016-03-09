#include "RedColor.h"

RedColor::RedColor()
{
    H_MIN = 0;
    H_MAX = 48;
    S_MIN = 93;
    S_MAX = 256;
    V_MIN = 141;
    V_MAX = 256;

    PRE_EROSIONS = 12;
    DILATIONS = 25;
    POST_EROSIONS = 5;
}

RedColor::~RedColor() {}