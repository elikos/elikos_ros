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
#include "internalModel/InternalModel.hpp"



namespace elikos_ai {

/**
 * @class	Agent Agent.hpp "Definition"
 * @brief	The AI agent making decisions for the quad.
 */
class Agent
{

public:

    Agent( ros::NodeHandle* nh );
    ~Agent();
    void init();  // start the agent (the elikos::ai node should do that)
    void destroy();
    void run();
    
    /* *************************************************************************************************
	 * ***           AI RELATED FUNCTIONS (DECISION-MAKING)
	 * *************************************************************************************************
	 */

	// Missing: the agent's internal model ->> environment, robots, obstacles
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


	/* *************************************************************************************************
     * ***           ROS SUBSCRIBERS CALLBACKS
     * *************************************************************************************************
     */

	// This needs to be public otherwise ROS won't be able to use the callback
    void receiveRobotsPosCallback( const elikos_ros::RobotsPos& msg );

    
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
    ros::NodeHandle& nh_;


    // ROS publishers, subscribers
    std::map<std::string, ros::Publisher> rosPublishers_; /**< @note map< topic name, the publisher object > */
    std::map<std::string, ros::Subscriber> rosSubscribers_; /**< @note map< topic name, the subscriber object > */

    // Test debug callback
    // ros::Subscriber sub_;

    /** @note   No pointer-reference to the message (RobotsPos instead of RobotsPos*)
     *          because the actual ROS's topic queue can dump out messages when it reaches
     *          its limit (the ROS's topic queue is where the original messages are from).
     *          Safer to make a copy of the message.
     */
    std::vector<elikos_ros::RobotsPos> queueRobotsPos_; /**< queue for robots positions from robot detection */


    // Internal model
    InternalModel* internalModel_;


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
    Agent& operator= (const Agent&);
    Agent (const Agent&);
};

} // namespace elikos_ai

#endif // AI_AGENT_H
