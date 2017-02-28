#include <ros/ros.h>
#include "MessageHandler.h"
#include "CmdExecutor.h"

int main(int argc, char* argv[])
{
    // ROS Init
    ros::init( argc, argv, "elikos_control" );

    CmdExecutor executor;


}
