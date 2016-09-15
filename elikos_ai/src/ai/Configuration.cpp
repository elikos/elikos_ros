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
    behaviorConfig_.isResearchEnabled = behaviorsConfig["research"]["enabled"].as<bool>();
    behaviorConfig_.isAggressiveEnabled = behaviorsConfig["aggressive"]["enabled"].as<bool>();
    behaviorConfig_.isPreventiveEnabled = behaviorsConfig["preventive"]["enabled"].as<bool>();
}

void Configuration::parseCommandsConfig(const YAML::Node &commandsConfig)
{
    cmdConfig_.moveAltitude = commandsConfig["move"]["altitude"].as<int>();
    cmdConfig_.followAltitude = commandsConfig["follow"]["altitude"].as<int>();
    cmdConfig_.topInteractionAltitude = commandsConfig["top_interaction"]["altitude"].as<int>();
    cmdConfig_.frontInteractionAltitude = commandsConfig["front_interaction"]["altitude"].as<int>();
}

void Configuration::parseConsiderationsConfig(const YAML::Node &considerationsConfig)
{
    considerationConfig_.isClusterSizeEnabled = considerationsConfig["cluster_size"]["enabled"].as<bool>();
    considerationConfig_.isQuadDistanceEnabled = considerationsConfig["quad_distance"]["enabled"].as<bool>();
    considerationConfig_.isTargetDestinationEnabled = considerationsConfig["target_destination"]["enabled"].as<bool>();
}

void Configuration::parseArenaConfig(const YAML::Node &arenaConfig)
{
    arenaConfig_.arenaType = arenaConfig["type"].as<char>();
    arenaConfig_.dimensions.setX(arenaConfig["dimensions"]["dx"].as<int>());
    arenaConfig_.dimensions.setY(arenaConfig["dimensions"]["dy"].as<int>());
}

}
