#include <ros/ros.h>
#include "MessageEmulator.h"

int main(int argc, char* argv[])
{
    // ROS Init
    ros::init(argc, argv, "elikos_ai");
    emu::MessageEmulator::getInstance()->lookForTf();
}
