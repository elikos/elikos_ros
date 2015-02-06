/**
* @brief    Artificial intelligence ROS node main program for Élikos project implementing an agent
*           behaviour.
*/

#include <ros/ros.h>

int main( int argc, char **argv )
{
    // Initialize the ROS system
    ros::init( argc, argv, "elikos_aiAgent" );

    // Establish this program as a ROS node
    ros::NodeHandle nh;

    // Send some output as a log message
    ROS_INFO_STREAM( "Hello, ai agent!" );
}