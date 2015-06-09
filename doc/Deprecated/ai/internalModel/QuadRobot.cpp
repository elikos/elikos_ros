/*
 * QuadRobot.cpp
 *
 *  Created on: 2015-03-06
 *      Author: myriam
 */

#include "QuadRobot.hpp"

namespace elikos_ai {

QuadRobot::QuadRobot( int id, tf::Point relativePosition, float orientation )
: Robot( id, relativePosition, orientation, tf::Vector3(0.0, 0.0, 0.0), quadRobot )
{}

QuadRobot::QuadRobot( int id, tf::Point relativePosition, float orientation, tf::Vector3 speed )
: Robot( id, relativePosition, orientation, speed, quadRobot )
{}

QuadRobot::~QuadRobot()
{
    // TODO Auto-generated destructor stub
}

} // namespace elikos_ai
