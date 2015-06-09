/*
 * RobotsFactory.hpp
 *
 *  Created on: 2015-02-27
 *      Author: Myriam
 */

#ifndef AI_ROBOTSFACTORY_HPP
#define AI_ROBOTSFACTORY_HPP

#include <map>
#include "./../../../defines.cpp"
#include "RobotFactoryBase.hpp"

namespace elikos_ai {

/**
 * @class   elikos_ai::RobotsFactory    RobotsFactory.hpp   "Definition"
 * @brief   Singleton able to create all of the different internal model's robots.
 * @note    About the singleton, see: http://come-david.developpez.com/tutoriels/dps/?page=Singleton
 */

class RobotsFactory
{
public:

    static RobotsFactory& Instance() { return instance_; }

    /* *************************************************************************************************
     * ***           PUBLIC FACTORY FUNCTIONS
     * *************************************************************************************************
     */

    Robot* newRobot( RobotType robotType, int id, tf::Point relativePosition, float orientation );
    Robot* newRobot( RobotType robotType, int id, tf::Point relativePosition, float orientation, tf::Vector3 speed );

private:

    static RobotsFactory instance_;

    RobotsFactory();
    ~RobotsFactory();

    std::map<RobotType, RobotFactoryBase*> factoriesMap_;

    /* *************************************************************************************************
     * ***           HIDDEN CONSTRUCTORS (do not implement)
     * *************************************************************************************************
     */

    RobotsFactory& operator= (const RobotsFactory&);
    RobotsFactory (const RobotsFactory&);
};

} // namespace elikos_ai

#endif // AI_ROBOTSFACTORY_HPP
