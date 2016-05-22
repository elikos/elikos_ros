#include <memory>
#include "MessageHandler_abs.h"

int main(int argc, char* argv[])
{
    // ROS Init
    ros::init( argc, argv, "elikos_tracking_abs" );
   
    MessageHandler_abs messageHandler;

    ros::Rate r(30);
    // Endless loop
    while(ros::ok()) 
    {
        ros::spinOnce();
        r.sleep();
    }
}
