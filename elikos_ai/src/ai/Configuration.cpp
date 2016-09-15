#include <cstring>
#include <ros/ros.h>
#include <thread>

#include "Configuration.h"

#include "Agent.h"

namespace ai
{

const char* Configuration::SIM_LAUNCH_CMD{"roslaunch elikos_sim simulation.launch"};
const char* Configuration::ARG_MODE{"--mode"};

Configuration::Configuration()
{
}

std::string Configuration::parseNodeArgs(int argc, char** argv)
{
    const std::string CONFIG_ARG = "--config";
    for (int i = 0; i < argc; i++)
    {
        int j = i + 1;
        if (!strcmp(CONFIG_ARG.c_str(), argv[i]) && j < argc)
        {
            readFromYamlFile(std::string(argv[j]));
        }
    }
}

bool Configuration::readFromYamlFile(const std::string &filePath)
{
    YAML::Node config = YAML::LoadFile(filePath);

    YAML::Node behaviors = config["behaviors"];
    parseBehaviorsConfig(behaviors);

    YAML::Node commands = config["commands"];
    parseCommandsConfig(commands);

    YAML::Node considerations = config["considerations"];
    parseConsiderationsConfig(considerations);

    YAML::Node arena = config["arena"];
    parseArenaConfig(arena);
}

void Configuration::parseBehaviorsConfig(const YAML::Node &behaviorsConfig)
{
    bool hasResearch =   behaviorsConfig["research"]["enabled"].as<bool>();
    bool hasAggressive = behaviorsConfig["aggressive"]["enabled"].as<bool>();
    bool hasPreventive = behaviorsConfig["preventive"]["enabled"].as<bool>();
}

void Configuration::parseCommandsConfig(const YAML::Node &commandsConfig)
{
    int moveHeight =   commandsConfig["move"]["altitude"].as<int>();
    int followHeight = commandsConfig["follow"]["altitude"].as<int>();
    int topHeight =    commandsConfig["top_interaction"]["altitude"].as<int>();
    int frontHeight =  commandsConfig["front_interaction"]["altitude"].as<int>();
}

void Configuration::parseConsiderationsConfig(const YAML::Node &considerationsConfig)
{
    bool isUsingClusterSize = considerationsConfig["cluster_size"]["enabled"].as<bool>();
    bool isUsingQuadDistance = considerationsConfig["quad_distance"]["enabled"].as<bool>();
    bool isUsingTargetDestination = considerationsConfig["target_destination"]["enabled"].as<bool>();
}

void Configuration::parseArenaConfig(const YAML::Node &arenaConfig)
{
    char arenaType =  arenaConfig["type"].as<char>();
    int dx = arenaConfig["dimensions"]["dx"].as<int>();
    int dy = arenaConfig["dimensions"]["dy"].as<int>();
}

}
