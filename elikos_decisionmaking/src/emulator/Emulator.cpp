#include <ros/ros.h>
#include "MessageEmulator.h"

int main(int argc, char* argv[])
{
    // ROS Init
    ros::init(argc, argv, "elikos_decisionmaking");
    emu::MessageEmulator::getInstance()->lookForTf();
}
