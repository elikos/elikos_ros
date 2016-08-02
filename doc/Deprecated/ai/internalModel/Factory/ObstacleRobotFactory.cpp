/*
 * ObstacleRobotFactory.cpp
 *
 *  Created on: 2015-02-27
 *      Author: myriam
 */

#include "ObstacleRobotFactory.hpp"

namespace elikos_ai {

ObstacleRobotFactory::ObstacleRobotFactory()
{}

ObstacleRobotFactory::~ObstacleRobotFactory()
{}

/* *************************************************************************************************
 * ***           INHERITED MEMBERS
 * *************************************************************************************************
 */
Robot* ObstacleRobotFactory::newRobot( int id, tf::Point relativePosition, float orientation )
{
    return new ObstacleRobot( id, relativePosition, orientation );
}

Robot* ObstacleRobotFactory::newRobot( int id, tf::Point relativePosition, float orientation, tf::Vector3 speed )
{
    return new ObstacleRobot( id, relativePosition, orientation, speed );
}

} // namespace elikos_ai
