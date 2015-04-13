/**
* @brief    Artificial intelligence ROS node main program for Élikos project implementing an agent
*           behaviour.
*/

#include <ros/ros.h>
#include "Agent.h"


int main( int argc, char **argv )
{
    // Initialize the ROS system
    ros::init( argc, argv, "elikos_aiAgent" );

    // Establish this program as a ROS node
    ros::NodeHandle nh;
    ros::Rate r(30);    //10 hz

    // Create the quad AI Agent
    elikos_ai::Agent agent(&nh);
    agent.init();

    while (ros::ok())
    {
        ros::spinOnce();

    	agent.percept(); // get the environnement's state
    	agent.chooseAction(); // choose the best action considering the environnement's current state
    	agent.executeAction(); // action!

	    r.sleep();
	}

    agent.destroy();
}

