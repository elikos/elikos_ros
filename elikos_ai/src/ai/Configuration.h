#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <string>
#include <unordered_map>
#include <memory>

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
    void parse(int argc, char* argv[]);
    void fetchParameters(const ros::NodeHandle& nh);

private:
    std::unordered_map<std::string, bool> enableFlags_;

    int checkModeArg(int argc, char* argv[]);
    void parseMode(int mode);
};

}

#endif /// CONFIGURATION_H
