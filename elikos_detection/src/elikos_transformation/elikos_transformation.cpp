#include <memory>
#include "MessageHandler.h"

int main(int argc, char* argv[])
{
    // ROS Init
    ros::init( argc, argv, "elikos_transformation" );

    MessageHandler messageHandler;

    ros::Rate r(30);
    // Endless loop
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}