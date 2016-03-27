#include <memory>

#include "MessageHandler_ttf.h"

int main(int argc, char* argv[])
{
    // ROS Init
    ros::init( argc, argv, "elikos_tracking_tf" );
   
    MessageHandler_TTF messageHandler;

    ros::Rate r(30);
    // Endless loop
    while(ros::ok()) 
    {
        ros::spinOnce();
        r.sleep();
    }
}
