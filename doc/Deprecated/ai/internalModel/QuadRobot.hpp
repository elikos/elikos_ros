/*
 * QuadRobot.hpp
 *
 *  Created on: 2015-03-06
 *      Author: myriam
 */

#ifndef AI_QUADROBOT_HPP
#define AI_QUADROBOT_HPP

#include "Robot.hpp"

namespace elikos_ai {

class QuadRobot : public Robot
{
public:
    virtual ~QuadRobot();
    QuadRobot( int id, tf::Point relativePosition, float orientation ); //constructor
    QuadRobot( int id, tf::Point relativePosition, float orientation, tf::Vector3 speed ); //constructor

private:
    /* *************************************************************************************************
     * ***           HIDDEN CONSTRUCTORS (do not implement)
     * *************************************************************************************************
     */

    QuadRobot();
    QuadRobot& operator= (const QuadRobot&);
    QuadRobot (const QuadRobot&);
}; // class elikos_ai::

} // namespace elikos_ai::QuadRobot

#endif // AI_QUADROBOT_HPP
