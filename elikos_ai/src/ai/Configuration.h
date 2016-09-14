#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <string>
#include <unordered_map>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace ai
{

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

private:
    std::unordered_map<std::string, bool> enableFlags_;
    void parseBehaviorsConfig(const YAML::Node& behaviorsConfig);
    void parseCommandsConfig(const YAML::Node& commandsConfig);
    void parseConsiderationsConfig(const YAML::Node& considerationsConfig);
    void parseArenaConfig(const YAML::Node& arenaConfig);

};

}

#endif /// CONFIGURATION_H
