/**
* @file		ai/internalModel/ObstacleRobot.cpp
* @author	Myriam Claveau-Mallet
* @date		02/08/2015
*/

#include "ObstacleRobot.hpp"

namespace elikos_ai {

ObstacleRobot::ObstacleRobot( int id, tf::Point relativePosition, float orientation )
: Robot( id, relativePosition, orientation, tf::Vector3(0.0, 0.0, 0.0), obstacleRobot )
{

}

ObstacleRobot::ObstacleRobot( int id, tf::Point relativePosition, float orientation, tf::Vector3 speed )
: Robot( id, relativePosition, orientation, speed, obstacleRobot )
{

}

} // namespace elikos_ai
