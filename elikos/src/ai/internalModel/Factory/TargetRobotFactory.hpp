/*
 * TargetRobotFactory.hpp
 *
 *  Created on: 2015-02-27
 *      Author: myriam
 */

#ifndef AI_TARGETROBOTFACTORY_HPP
#define AI_TARGETROBOTFACTORY_HPP

#include "RobotFactoryBase.hpp"
#include "./../TargetRobot.hpp"

namespace elikos_ai {

class TargetRobotFactory : public RobotFactoryBase
{
public:
    TargetRobotFactory();
    virtual ~TargetRobotFactory();

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

    TargetRobotFactory& operator= (const TargetRobotFactory&);
    TargetRobotFactory (const TargetRobotFactory&);
}; // class elikos_ai::TargetRobotFactory

} // namespace elikos_ai

#endif // AI_TARGETROBOTFACTORY_HPP
