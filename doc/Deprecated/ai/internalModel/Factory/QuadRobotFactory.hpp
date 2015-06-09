/*
 * QuadRobotFactory.hpp
 *
 *  Created on: 2015-03-06
 *      Author: myriam
 */

#ifndef AI_QUADROBOTFACTORY_HPP
#define AI_QUADROBOTFACTORY_HPP

#include "RobotFactoryBase.hpp"

namespace elikos_ai {

/**
 * @class   QuadRobotFactory QuadRobotFactory.hpp "Definition"
 * @brief   The AI'S internal model's robots' factory for the quad.
 * @note    The base class never gets instantiated, only the children concrete classes, like this one.
 * @note    The robots' factories are contained in another class, RobotsFactory. They are never
 *          accessed directly in the code, only via this other class, RobotsFactory.
 */
class QuadRobotFactory : public RobotFactoryBase
{
public:
    QuadRobotFactory();
    virtual ~QuadRobotFactory();

    /* *************************************************************************************************
     * ***           INHERITED MEMBERS
     * *************************************************************************************************
     */
    virtual Robot* newRobot( int id, tf::Point relativePosition, float orientation );
    virtual Robot* newRobot( int id, tf::Point relativePosition, float orientation, tf::Vector3 speed );

private:
    /* *************************************************************************************************
     * ***           HIDDEN CONSTRUCTORS (do not implement)
     * *************************************************************************************************
     */

    QuadRobotFactory& operator= (const QuadRobotFactory&);
    QuadRobotFactory (const QuadRobotFactory&);
}; // class elikos_ai::QuadRobotFactory

} // namespace elikos_ai

#endif // AI_QUADROBOTFACTORY_HPP
