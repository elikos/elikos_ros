/**
* @file		ai/internalModel/Robot.cpp
* @author	Myriam Claveau-Mallet
* @date		02/08/2015
*/

#include "Robot.hpp"

namespace elikos_ai {

Robot::Robot ( int id, tf::Vector3 relativePosition, float orientation, tf::Vector3 speed, RobotType type )
: id_(id), orientation_(orientation), speed_(speed), type_(type)
{
    // TODO: convertir la position relative dans une position absolue (par rapport au quad) dans this.transform_
    transform_.setOrigin( relativePosition );
}

void Robot::updateRelativePosition( tf::Vector3 relativePosition, float orientation )
{
    // TODO: retenir les vieilles positions et orientations
    transform_.setOrigin( relativePosition );
    orientation_ = orientation;
}

} // namespace elikos_ai

