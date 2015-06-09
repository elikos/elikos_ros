/*
 * QuadRobotFactory.cpp
 *
 *  Created on: 2015-03-06
 *      Author: myriam
 */

#include "QuadRobotFactory.hpp"
#include "../QuadRobot.hpp"

namespace elikos_ai {

QuadRobotFactory::QuadRobotFactory()
{}

QuadRobotFactory::~QuadRobotFactory()
{}


/* *************************************************************************************************
 * ***           INHERITED MEMBERS
 * *************************************************************************************************
 */
Robot* QuadRobotFactory::newRobot( int id, tf::Point relativePosition, float orientation )
{
    return new QuadRobot( id, relativePosition, orientation );
}

Robot* QuadRobotFactory::newRobot( int id, tf::Point relativePosition, float orientation, tf::Vector3 speed )
{
    return new QuadRobot( id, relativePosition, orientation, speed );
}

} // namespace elikos_ai
