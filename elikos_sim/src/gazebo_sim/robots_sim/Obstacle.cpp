#include "Obstacle.h"

Obstacle::Obstacle(std::string name, double x, double y, double speed, double updateRate, ros::ServiceClient client): Robot(name, x, y, speed, updateRate, client)
{
    _radius = sqrt(pow(x,2)+pow(y,2));
}

void Obstacle::move()
{
    _setmodelstate.request.model_state.pose.position.x = _radius*cos(_angle);
    _setmodelstate.request.model_state.pose.position.y = _radius*sin(_angle);
    
    _client.call(_setmodelstate);

    _angle += _speed/(_radius*_updateRate);
}