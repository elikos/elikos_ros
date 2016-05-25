#include <cstring>
#include <ros/ros.h>
#include <thread>

#include "MsgEmulator.h"

#include "CmdLineParser.h"

namespace ai
{

const std::string CmdLineParser::PARAM_SIM = "--sim";

CmdLineParser::CmdLineParser(int argc, char* argv[])
    : argc_(argc), argv_(argv)
{
}


void CmdLineParser::parse()
{
    bool isSimulation = checkParam(PARAM_SIM);
    if(isSimulation)
    {
        launchSimulation();
    }
}

/*
 *  bool CmdLineParser::checkParam(const std::string& param)
 *
 *  Check if one of the input arguments is valid with a given parameter.
 *
 *  param[in] : parameter to check for in the arguments
 *  return : if the parameter has been found
 */
bool CmdLineParser::checkParam(const std::string& param)
{
    for (int i = 0; i < argc_; i++)
    {
        if(strcmp(argv_[i], param.c_str()) == 0)
        {
            return true;
        }
    }
}

void CmdLineParser::launchSimulation()
{
    //TODO: launch sim here.
    bool success = MsgEmulator::getInstance()->start();
    if(!success)
    {
        std::cout << "Error: Failed to start msg emulator." << std::endl;
    }
}

}
