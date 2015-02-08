/*
 * Agent.h
 *
 *  Created on: Jan 10, 2015
 *      Author: Myriam Claveau-Mallet
 */

#ifndef __AGENT_H_
#define __AGENT_H_

#include <ros/ros.h>
#include <vector>
#include <string>

namespace elikos_ai {

class Agent
{

public:

    Agent();
    void init();  // start the agent (the elikos::ai node should do that)
    void destroy();
    void run();
    
private:
    // AI RELATED FUNCTIONS
    // Missing: the agent's internal model ->> envionment, robots, obstacles
    // Missing: the agent's possible actions
    // Missing: the agent's rules of action and utility decision's rules

    // Missing: the agent's sequence of planned actions (a vector or something)

    // For now, the following functions are only ideas. It's still all a sketch.
    // The order in which they are now written (still as a sketch) is meant
    // to represent the order in which they will be called. This order is also
    // a sketch and might change.

    void percept();       // get data from other ROS nodes (topics and services)
    void updateModel();

    void chooseAction();
    void predictImpactActionOnModel();
    void evaluateUtility();

    void executeAction(); // communicate with ROS

    
    bool actionIsDone();

    // Missing: loop of functions' calls (in some "run" function)


    // Publishers and subscribers
    void setPublishers();
    void setSubscribers();

    void removePublishers();
    void removeSubscribers();

    // Attributes
    /*std::string Name;
    ros::Duration cycleTime;
    tf::Vector3 direction;
    tf::Transform t;
    tf::Vector3 v;
    tf::Quaternion q;*/

    // Attributes : publishers, subscribers
    ros::Publisher pub_orders;
    std::vector<ros::Subscriber> subs;


    // Hidden constructors

    Agent( const Agent& agent  ); // do not implement
};

} // namespace elikos_ai

#endif // __AGENT_H_
