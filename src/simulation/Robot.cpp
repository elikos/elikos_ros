#include "Robot.hpp"

void Robot::refreshTransform(){
    v.setX(x);
    v.setY(y);
    v.setZ(z);
    q.setRPY(0, 0, yaw);
    transform.setOrigin(v);
    transform.setRotation(q);
}
