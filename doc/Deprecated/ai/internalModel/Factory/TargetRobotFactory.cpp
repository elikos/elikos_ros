/*
 * TargetRobotFactory.cpp
 *
 *  Created on: 2015-02-27
 *      Author: myriam
 */

#include "TargetRobotFactory.hpp"

namespace elikos_ai {

TargetRobotFactory::TargetRobotFactory()
{}

TargetRobotFactory::~TargetRobotFactory()
{}


/* *************************************************************************************************
 * ***           INHERITED MEMBERS
 * *************************************************************************************************
 */
Robot* TargetRobotFactory::newRobot( int id, tf::Point relativePosition, float orientation )
{
    return new TargetRobot( id, relativePosition, orientation );
}

Robot* TargetRobotFactory::newRobot( int id, tf::Point relativePosition, float orientation, tf::Vector3 speed )
{
    return new TargetRobot( id, relativePosition, orientation, speed );
}

} // namespace elikos_ai
