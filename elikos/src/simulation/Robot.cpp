#include "Robot.hpp"

Robot::Robot(int id, double simSpeed) : id(id), simSpeed(simSpeed) {
    x = 0;
    y = 0;
    z = 0;
    yaw = 0;
    turnAngle = 0;
}

void Robot::refreshTransform(){
    v.setX(x);
    v.setY(y);
    v.setZ(z);
    q.setRPY(0, 0, yaw);
    transform.setOrigin(v);
    transform.setRotation(q);
}
