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

    inline bool getIsSimulation();

    void parse();

private:
    int argc_;
    char** argv_;
    bool isSimulation_{ false };

    bool checkParam(const std::string& param);
    void launchSimulation();


};

inline bool CmdLineParser::getIsSimulation()
{
    return isSimulation_;
}

}

#endif /// CMD_LINE_PARSER
