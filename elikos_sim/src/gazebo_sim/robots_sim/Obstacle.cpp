#include "Obstacle.h"

Obstacle::Obstacle(std::string name, double x, double y, double speed, double updateRate, ros::ServiceClient client): Robot(name, x, y, speed, updateRate, client)
{
    _radius = sqrt(pow(x,2)+pow(y,2));
}

void Obstacle::move()
{

}