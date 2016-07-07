#include "AbstractRobot.h"

namespace ai {

const double AbstractRobot::SPEED = 0.2;

AbstractRobot::~AbstractRobot()
{
}

void AbstractRobot::updatePositionRadius(const double& dt)
{
    // TODO: use stats to be more accurate.
    positionRadius_ += SPEED * dt;
}

tfScalar AbstractRobot::getDistance(AbstractRobot* robot)
{
    return tf::tfDistance(pose_.getOrigin(), robot->getPose().getOrigin());
}

};
