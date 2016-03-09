#include "WhiteColor.h"

WhiteColor::WhiteColor()
{
    H_MIN = 0;
    H_MAX = 52;
    S_MIN = 0;
    S_MAX = 149;
    V_MIN = 140;
    V_MAX = 256;

    PRE_EROSIONS = 12;
    DILATIONS = 25;
    POST_EROSIONS = 5;
}

WhiteColor::~WhiteColor() {}

void WhiteColor::printFoundObjects()
{
    // Not interesting.
}

void WhiteColor::drawFoundObjects(Mat& image)
{
    // Not interesting.
}