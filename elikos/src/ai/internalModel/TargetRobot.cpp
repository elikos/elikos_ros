/**
* @file		ai/internalModel/TargetRobot.cpp
* @author	Myriam Claveau-Mallet
* @date		02/08/2015
*/

#include "TargetRobot.hpp"

namespace elikos_ai {

TargetRobot::TargetRobot( int id, tf::Point relativePosition, float orientation )
: Robot( id, relativePosition, orientation, tf::Vector3(0.0, 0.0, 0.0), groundRobot )
{

}

TargetRobot::TargetRobot( int id, tf::Point relativePosition, float orientation, tf::Vector3 speed )
: Robot( id, relativePosition, orientation, speed, groundRobot )
{

}

} // namespace elikos_ai
