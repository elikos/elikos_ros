/*
 * Agent.h
 *
 *  Created on: Jan 10, 2015
 *      Author: Myriam Claveau-Mallet
 */

#ifndef AI_AGENT_H
#define AI_AGENT_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <map>
#include <string>
#include <cmath>

#include <elikos_ros/RobotsPos.h>

#include "Action.hpp"



namespace elikos_ai {

/**
 * @class	Agent Agent.hpp "Definition"
 * @brief	The AI agent making decisions for the quad.
 */
class Agent
{

public:

    Agent( ros::NodeHandle *nh );
    ~Agent();
    void init();  // start the agent (the elikos::ai node should do that)
    void destroy();
    void run();
    
    /* *************************************************************************************************
	 * ***           AI RELATED FUNCTIONS (DECISION-MAKING)
	 * *************************************************************************************************
	 */

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

    
private:




	/* *************************************************************************************************
     * ***           ROS RELATED FUNCTIONS
     * *************************************************************************************************
     */

    // Publishers and subscribers
    void setPublishers();
    void setSubscribers();

    void removePublishers();
    void removeSubscribers();

    void receiveRobotsPos( const elikos_ros::RobotsPos& msg );

	/* *************************************************************************************************
     * ***           TOOLS
     * *************************************************************************************************
     */

    geometry_msgs::PoseStamped getPoseStamped(float angle);


    /* *************************************************************************************************
     * ***           ATTRIBUTES
     * *************************************************************************************************
     */

    // ROS node handle
    ros::NodeHandle* nh_;

    // Attributes
    /*std::string Name;
    ros::Duration cycleTime;
    tf::Vector3 direction;
    tf::Transform t;
    tf::Vector3 v;
    tf::Quaternion q;*/

    // ROS publishers, subscribers
    std::map<std::string, ros::Publisher> rosPublishers_; // map< topic name, the publisher object >
    std::map<std::string, ros::Subscriber> rosSubscribers_; // map< topic name, the subscriber object >

    // List of actions planned by the agent
    std::vector<Action> actions_;

    // Temp action for quick testing
    Action* action_;
    float angle_;


    /* *************************************************************************************************
     * ***           HIDDEN CONSTRUCTORS (do not implement)
     * *************************************************************************************************
     */

    Agent();
    Agent( const Agent& agent  );
};

} // namespace elikos_ai

#endif // AI_AGENT_H
