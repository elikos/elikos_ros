#ifndef OBSTACLE_H
#define OBSTACLE_H
#include "Robot.h"
class Obstacle : public Robot
{
public:
    Obstacle(std::string name, double x, double y, double speed, double updateRate, ros::ServiceClient client);

    virtual void move();

private:
    double _radius;
};
#endif