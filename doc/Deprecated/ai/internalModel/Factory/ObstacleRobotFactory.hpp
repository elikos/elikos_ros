/*
 * ObstacleRobotFactory.hpp
 *
 *  Created on: 2015-02-27
 *      Author: myriam
 */

#ifndef AI_OBSTACLEROBOTFACTORY_HPP
#define AI_OBSTACLEROBOTFACTORY_HPP

#include "RobotFactoryBase.hpp"
#include "./../ObstacleRobot.hpp"

namespace elikos_ai {

/**
 * @class   ObstacleRobotFactory ObstacleRobotFactory.hpp "Definition"
 * @brief   The AI'S internal model's robots' factory for obstacle robots.
 * @note    This base class never gets instantiated, only the children concrete classes, like this one.
 * @note    The robots' factories are contained in another class, RobotsFactory. They are never
 *          accessed directly in the code, only via this other class, RobotsFactory.
 */
class ObstacleRobotFactory : public RobotFactoryBase
{
public:
    ObstacleRobotFactory();
    virtual ~ObstacleRobotFactory();

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

    ObstacleRobotFactory& operator= (const ObstacleRobotFactory&);
    ObstacleRobotFactory (const ObstacleRobotFactory&);
}; // class elikos_ai::ObstacleRobotFactory

} // namespace elikos_ai

#endif // AI_OBSTACLEROBOTFACTORY_HPP
