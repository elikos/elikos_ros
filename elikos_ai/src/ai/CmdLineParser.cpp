#include <cstring>
#include <ros/ros.h>
#include <thread>

#include "MessageEmulator.h"

#include "CmdLineParser.h"

namespace ai
{

const std::string CmdLineParser::PARAM_SIM = "--sim";
const std::string CmdLineParser::SIM_LAUNCH_CMD = "roslaunch elikos_sim simulation.launch";

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
    bool foundParam = false;
    for (int i = 0; i < argc_ && !foundParam; i++)
    {
        if(strcmp(argv_[i], param.c_str()) == 0)
        {
            foundParam = true;
        }
    }
    return foundParam;
}

/*
 *  void CmdLineParser::launchSimulation()
 *
 *  Launch sim and msg emulator
 */
void CmdLineParser::launchSimulation()
{
    //std::system(SIM_LAUNCH_CMD.c_str());
    bool success = MessageEmulator::getInstance()->start();
    if(!success)
    {
        std::cout << "Error: Failed to start msg emulator." << std::endl;
    }
}

}
