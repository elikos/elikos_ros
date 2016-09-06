#include "MessageHandler.h"
#include "Agent.h"
#include "Configuration.h"

int main(int argc, char* argv[])
{
    // ROS Init
    ros::init(argc, argv, "elikos_ai");

    ros::NodeHandle nh;
    ai::Configuration config;
    config.fetchParameters(nh);

    ai::Agent::getInstance()->init(&config);
    ai::MessageHandler::getInstance()->lookForMessages();

    ai::MessageHandler::freeInstance();
    ai::Agent::freeInstance();
}
