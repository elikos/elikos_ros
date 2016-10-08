//
// Created by olivier on 10/2/16.
//

#include "Line.h"

namespace localization
{

Line::Line(float rho, float theta)
    : orientation_(cosf(theta), sinf(theta)), rho_(rho), theta_(theta)
{
}

void Line::rotate(double rotation)
{
    theta_ += rotation;
    orientation_ = { cosf(theta_), sinf(theta_) };
}

void Line::inverseOrientation()
{
    theta_ += CV_PI;
    orientation_ = { cosf(theta_), sinf(theta_) };
}

};
