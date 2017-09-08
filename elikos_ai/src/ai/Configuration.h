#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <string>
#include <unordered_map>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <tf/tf.h>

namespace ai
{

struct BehaviorConfig
{
    bool isResearchEnabled;
    bool isAggressiveEnabled;
    bool isPreventiveEnabled;
};

struct ConsiderationConfig
{
    bool isClusterSizeEnabled;
    bool isQuadDistanceEnabled;
    bool isTargetDestinationEnabled;
};

struct ArenaConfig
{
    char arenaType;
    tf::Vector3 dimensions;
};

struct CommandConfig
{
    double moveAltitude;
    double followAltitude;
    double frontInteractionAltitude;
    double topInteractionAltitude;
};

class AbstractConsideration;

class Configuration
{
public:
    static const char* SIM_LAUNCH_CMD;
    static const char* ARG_SIM;
    static const char* ARG_MODE;

    Configuration();
    std::string parseNodeArgs(int argc, char** argv);
    bool readFromYamlFile(const std::string& path);

    inline ArenaConfig* getArenaConfig();
    inline BehaviorConfig* getBehaviorConfig();
    inline CommandConfig* getCommandConfig();
    inline ConsiderationConfig* getConsiderationConfig();

private:
    std::unordered_map<std::string, bool> enableFlags_;
    void parseBehaviorsConfig(const YAML::Node& behaviorsConfig);
    void parseCommandsConfig(const YAML::Node& commandsConfig);
    void parseConsiderationsConfig(const YAML::Node& considerationsConfig);
    void parseArenaConfig(const YAML::Node& arenaConfig);

    ArenaConfig arenaConfig_;
    BehaviorConfig behaviorConfig_;
    CommandConfig cmdConfig_;
    ConsiderationConfig considerationConfig_;
};

inline ArenaConfig* Configuration::getArenaConfig()
{
   return &arenaConfig_;
}

inline BehaviorConfig* Configuration::getBehaviorConfig()
{
   return &behaviorConfig_;
}

inline CommandConfig* Configuration::getCommandConfig()
{
   return &cmdConfig_;
}

inline ConsiderationConfig* Configuration::getConsiderationConfig()
{
   return &considerationConfig_;
}

}

#endif /// CONFIGURATION_H
