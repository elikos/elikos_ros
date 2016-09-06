#include <cstring>
#include <ros/ros.h>
#include <thread>

#include "Configuration.h"

#include "Agent.h"

namespace ai
{

const char* Configuration::SIM_LAUNCH_CMD{ "roslaunch elikos_sim simulation.launch" };
const char* Configuration::ARG_MODE { "--mode" };

Configuration::Configuration()
{
}

void Configuration::fetchParameters(const ros::NodeHandle& nh)
{
}

}
