#include "Robot.h"

namespace ai {

const double Robot::SPEED = 0.2;

Robot::~Robot()
{
}

void Robot::updatePositionRadius(const double& dt)
{
    // TODO: use stats to be more accurate.
    positionRadius_ += SPEED * dt;
}

tfScalar Robot::getDistance(Robot* robot)
{
    return tf::tfDistance(pose_.getOrigin(), robot->getPose().getOrigin());
}

};
