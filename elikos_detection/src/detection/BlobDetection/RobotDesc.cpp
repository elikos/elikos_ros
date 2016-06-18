#include "RobotDesc.h"
/*
 * Constructors and destructors
 */
RobotDesc::RobotDesc():direction(0), id(0), alreadyFound(false), insideFOV(true)
{
}

RobotDesc::RobotDesc(int id, int x, int y): radius(id), xPos(x), yPos(y), direction(0), id(0), alreadyFound(false), insideFOV(true)
{
}


RobotDesc::~RobotDesc()
{
}
/*
 * Setters and getters
 */
int RobotDesc::getID() const
{
    return id;
}

void RobotDesc::setID(int ID)
{
    id = ID;
}

double RobotDesc::getDirection() const{
    return direction;
}

void RobotDesc::setDirection(double DIRECTION){
    direction = DIRECTION;
}


cv::RotatedRect RobotDesc::getWindow() const{
    return window;
}

void RobotDesc::setWindow(cv::RotatedRect w){
    window =w;
}

int RobotDesc::getHPos()
{
    return xPos;
}

int RobotDesc::getVPos()
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

void RobotDesc::setRadius(int r)
{
    radius = r;
    area = PI*pow(r,2);
}

void RobotDesc::setColor(ColorsIndex c)
{
    color =c;
}

ColorsIndex RobotDesc::getColor() const{
    return color;
}

int RobotDesc::getRadius() const{
    return radius;
}

int RobotDesc::getXPos() const{
    return xPos;
}

int RobotDesc::getYPos() const{
    return yPos;
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

double  RobotDesc::getArea(){
    return area;
}

void  RobotDesc::setArea(double a){
    area = a;
}

