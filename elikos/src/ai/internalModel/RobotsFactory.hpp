/*
 * RobotsFactory.hpp
 *
 *  Created on: 2015-02-27
 *      Author: Myriam
 */

#ifndef ROBOTSFACTORY_HPP_
#define ROBOTSFACTORY_HPP_

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

private:


    static RobotsFactory instance_;

    RobotsFactory();
    ~RobotsFactory();


    /* *************************************************************************************************
     * ***           HIDDEN CONSTRUCTORS (do not implement)
     * *************************************************************************************************
     */

    RobotsFactory& operator= (const RobotsFactory&);
    RobotsFactory (const RobotsFactory&);
};

} // namespace elikos_ai

#endif // ROBOTSFACTORY_HPP_
