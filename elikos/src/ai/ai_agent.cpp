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
    int lol = 0;

    while (ros::ok())
    {
        ros::spinOnce();

        std::cout << "percept " << lol++ << "\n";
    	agent.percept(); // get the environnement's state
        std::cout << "choose " << lol++ << "\n";
    	agent.chooseAction(); // choose the best action considering the environnement's current state
        std::cout << "execute " << lol++ << "\n";
    	agent.executeAction(); // action!
        std::cout << "done " << lol++ << "\n";


	r.sleep();
	}

    agent.destroy();
}

