/*
 * RobotFactoryBase.hpp
 *
 *  Created on: 2015-02-27
 *      Author: myriam
 */

#ifndef AI_ROBOTFACTORYBASE_HPP
#define AI_ROBOTFACTORYBASE_HPP

#include "./../Robot.hpp"

namespace elikos_ai {

class RobotFactoryBase
{
public:
    virtual ~RobotFactoryBase();

    /* *************************************************************************************************
     * ***           VIRTUAL PURE
     * *************************************************************************************************
     */
    virtual Robot* newRobot( int id, tf::Point relativePosition, float orientation ) = 0;
    virtual Robot* newRobot( int id, tf::Point relativePosition, float orientation, tf::Vector3 speed ) = 0;

protected:
    RobotFactoryBase() {}

private:
    /* *************************************************************************************************
     * ***           HIDDEN CONSTRUCTORS (do not implement)
     * *************************************************************************************************
     */

    RobotFactoryBase& operator= (const RobotFactoryBase&);
    RobotFactoryBase (const RobotFactoryBase&);
}; // class elikos_ai::RobotFactoryBase

} // namespace elikos_ai

#endif // AI_ROBOTFACTORYBASE_HPP
