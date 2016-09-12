#include "MessageHandler.h"
#include "Agent.h"
#include "Configuration.h"

std::string parseConfigFilePath(int argc, char* argv[])
{
    const std::string CONFIG_ARG = "--config";
    for (int i = 0; i < argc; i++) {
        int j = i + 1;
        if (!strcmp(CONFIG_ARG.c_str(), argv[i]) && j < argc) {
            return std::string(argv[j]);
        }
    }
}

int main(int argc, char* argv[])
{
    // ROS Init
    ros::init(argc, argv, "elikos_ai");


    std::string configFilePath = parseConfigFilePath(argc, argv);

    ai::Configuration config;
    config.readFromYamlFile(configFilePath);

    ai::Agent::getInstance()->init(&config);
    ai::MessageHandler::getInstance()->lookForMessages();

    ai::MessageHandler::freeInstance();
    ai::Agent::freeInstance();
}
