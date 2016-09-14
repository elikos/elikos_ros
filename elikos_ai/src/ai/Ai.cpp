#include "MessageHandler.h"
#include "Agent.h"
#include "Configuration.h"

int main(int argc, char* argv[])
{
    // ROS Init
    ros::init(argc, argv, "elikos_ai");

    ai::Configuration config;
    config.parseNodeArgs(argc, argv);

    ai::Agent::getInstance()->init(&config);
    ai::MessageHandler::getInstance()->lookForMessages();

    ai::MessageHandler::freeInstance();
    ai::Agent::freeInstance();
}
