/*
 * Agent.cpp
 *
 *  Created on: Jan 10, 2015
 *      Author: Myriam Claveau-Mallet
 */

#include "Agent.h"
#include "./../defines.cpp"

namespace elikos_ai {

Agent::Agent()
{

}

void Agent::init()
{
    setPublishers();
    setSubscribers();
}

void Agent::destroy()
{
    //removePublishers();
    //removeSubscribers();
}

///
/// Create and setup publishers
///
void Agent::setPublishers()
{
    // Orders given to MavROS

}

///
/// Create and setup subscribers
///
void Agent::setSubscribers()
{
    // Subscribe to all robots' positions' topics
}

} // namespace elikos_ai
