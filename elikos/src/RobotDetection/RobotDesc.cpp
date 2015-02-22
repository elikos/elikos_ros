#include "RobotDesc.h"

RobotDesc::RobotDesc()
{
}

RobotDesc::~RobotDesc()
{
}

int RobotDesc::getXPos()
{
    return xPos;
}

int RobotDesc::getYPos()
{
    return yPos;
}

void RobotDesc::setXPos(int pos)
{
    xPos = pos;
}

void RobotDesc::setYPos(int pos)
{
    yPos = pos;
}

cv::Scalar RobotDesc::getHSVmax()
{
    return HSVmax;
}

cv::Scalar RobotDesc::getHSVmin()
{
    return HSVmin;
}

void RobotDesc::setHSVmax(cv::Scalar scalar)
{
    HSVmax = scalar;
}

void RobotDesc::setHSVmin(cv::Scalar scalar)
{
    HSVmin = scalar;
}