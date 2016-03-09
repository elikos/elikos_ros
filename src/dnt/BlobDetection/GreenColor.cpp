#include "GreenColor.h"

GreenColor::GreenColor()
{
    H_MIN = 39;
    H_MAX = 88;
    S_MIN = 54;
    S_MAX = 256;
    V_MIN = 30;
    V_MAX = 256;

    PRE_EROSIONS = 3;
    DILATIONS = 20;
    POST_EROSIONS = 0;
}

GreenColor::~GreenColor() {}