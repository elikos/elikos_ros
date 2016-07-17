#ifndef CMD_LINE_PARSER_H
#define CMD_LINE_PARSER_H

#include <string>
#include <unordered_map>
#include <memory>

namespace ai
{

class AbstractConsideration;

class CmdLineParser
{
public:
    static const char* SIM_LAUNCH_CMD;
    static const char* ARG_SIM;
    static const char* ARG_MODE;

    // 0001
    static const int TARGET_DESTINATION_MASK { 1 };
    // 0010
    static const int QUAD_DISTANCE_MASK { 2 };
    // 0100
    static const int CLUSTER_SIZE_MASK { 4 };

    CmdLineParser() = delete;
    CmdLineParser(int argc, char* argv[]);
    ~CmdLineParser() = default;

    void parse();

private:
    std::unordered_map<char, bool> args;
    int argc_;
    char** argv_;

    bool checkSimArg();
    int checkModeArg();
    void parseMode(int mode);
    void launchSimulation();
};

}

#endif /// CMD_LINE_PARSER
