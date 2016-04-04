#include <ros/ros.h>
#include <ros/package.h>
#include "MessageHandler.h"
#include <memory>

int main(int argc, char* argv[])
{
    // ROS Init
    ros::init( argc, argv, "elikos_ai" );
    ros::Rate r(30);
 
    ai::MessageHandler mh;

    // Endless loop
    while(ros::ok()) 
    {
        mh.lookupTransform();
        ros::spinOnce();
        r.sleep();
    }

    // Free ressources.
    AIFacade::freeInstance();
}
