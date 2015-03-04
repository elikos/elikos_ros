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

/**
 * @class   RobotFactoryBase RobotFactoryBase.hpp "Definition"
 * @brief   The base class for the AI'S internal model's robots' factories.
 * @note    This base class never gets instantiated, only the children concrete classes.
 * @note    The robots' factories are contained in another class, RobotsFactory. They are never
 *          accessed directly in the code, only via this other class, RobotsFactory.
 */
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
    RobotFactoryBase() {} /**< @note Only concrete classes should be instantiated. The constructor
                                     is therefore unaccessible except for the children classes. > */

private:
    /* *************************************************************************************************
     * ***           HIDDEN CONSTRUCTORS (do not implement)
     * *************************************************************************************************
     */

    RobotFactoryBase& operator= (const RobotFactoryBase&); /**< @note Do not implement. > */
    RobotFactoryBase (const RobotFactoryBase&); /**< @note Do not implement. > */
}; // class elikos_ai::RobotFactoryBase

} // namespace elikos_ai

#endif // AI_ROBOTFACTORYBASE_HPP
