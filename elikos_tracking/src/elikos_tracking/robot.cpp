#include "robot.h"
#include <cmath>

Robot::Robot(int id, uint8_t color) {
    this->id = id;
    this->color = color;
    this->isNew = true;
    // this->incertitude = -1;
    this->incertitude = 100;
    this->speed = 0.1;
    this->isAssigned = false;
}

Robot::~Robot() {}

double Robot::getDistanceFrom(geometry_msgs::Point pos) {
    return sqrt(pow(pos.x - this->poseOrigin.x, 2) +
                pow(pos.y - this->poseOrigin.y, 2));
}

void Robot::setPos(geometry_msgs::Point point) {
    this->poseOrigin = point;
    // this->poseOrigin.header.stamp = ros::Time::now();
    this->incertitude = 1.0;
}

geometry_msgs::Point Robot::getPos() { return this->poseOrigin; }

void Robot::setColor(uint8_t color) { this->color = color; }

uint8_t Robot::getColor() { return this->color; }

double Robot::getIncertitude() { return this->incertitude; }

void Robot::setSpeed(double speed) { this->speed = speed; }

double Robot::getSpeed() { return this->speed; }

double Robot::updateIncertitude(int dt) {
    // si dt est en nanosecondes
    this->incertitude += (float)(dt / pow(10, 9)) * this->speed;
   // ROS_INFO("Incertitude for %d is now %f", this->id, this->incertitude);
    return this->incertitude;
}