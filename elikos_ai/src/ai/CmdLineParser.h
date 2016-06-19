#ifndef CMD_LINE_PARSER_H
#define CMD_LINE_PARSER_H

#include <string>

namespace ai
{

class CmdLineParser
{
public:
    static const std::string PARAM_SIM;

    CmdLineParser() = delete;
    CmdLineParser(int argc, char* argv[]);
    ~CmdLineParser() = default;

    void parse();

private:
    int argc_;
    char** argv_;

    bool checkParam(const std::string& param);
    void launchSimulation();
};

}

#endif /// CMD_LINE_PARSER
