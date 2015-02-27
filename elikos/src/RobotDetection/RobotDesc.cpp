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

elikos_ros::RobotPos RobotDesc::toMsg()
{
    elikos_ros::RobotPos robotInfo;
    int zPos = 0;
    geometry_msgs::Point point;

    point.x = xPos;
    point.y = yPos;
    point.z = zPos;

    robotInfo.id = 1;
    robotInfo.isNew = true;
    robotInfo.point = point;
    robotInfo.orientation = 0;

    return robotInfo;
}