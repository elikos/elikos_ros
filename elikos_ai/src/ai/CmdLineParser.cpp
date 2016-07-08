#include <cstring>
#include <ros/ros.h>
#include <thread>

#include "CmdLineParser.h"

#include "MessageEmulator.h"
#include "Agent.h"
#include "ConsiderationTypes.h"

namespace ai
{

const char* CmdLineParser::SIM_LAUNCH_CMD{ "roslaunch elikos_sim simulation.launch" };
const char* CmdLineParser::ARG_SIM  { "--sim" };
const char* CmdLineParser::ARG_MODE { "--mode" };

CmdLineParser::CmdLineParser(int argc, char* argv[])
    : argc_(argc), argv_(argv)
{
}


void CmdLineParser::parse()
{
    bool isSimulation = checkSimArg();
    if(isSimulation)
    {
        launchSimulation();
    }

    int mode = checkModeArg();
    if (mode > 0)
    {
        parseMode(mode);
    }
}

bool CmdLineParser::checkSimArg()
{
    bool foundParam = false;
    for (int i = 0; i < argc_ && !foundParam; ++i)
    {
        if (strcmp(argv_[i], ARG_SIM) == 0)
        {
            foundParam = true;
        }
    }
    return foundParam;
}

int CmdLineParser::checkModeArg()
{
    bool foundParam = false;
    int mode = 0;
    for (int i = 0; i < argc_ && !foundParam; ++i)
    {
        if (strcmp(argv_[i], ARG_MODE) == 0)
        {
            if (i + 1 < argc_)
            {
                mode = std::stoi(argv_[i + 1]);
                foundParam = true;
            }
        }
    }
    return mode;
}

void CmdLineParser::parseMode(int mode)
{
    PriorityEvaluationPipeline* pipeline = Agent::getInstance()->getConsiderationPipeline();

    if ((mode & RED_LINE_MASK) == RED_LINE_MASK)
    {
        pipeline->addConsideration(std::unique_ptr<TargetDestination>(new TargetDestination(pipeline->getArena())));
    }

    if ((mode & QUAD_DISTANCE_MASK) == QUAD_DISTANCE_MASK)
    {
        pipeline->addConsideration(std::unique_ptr<QuadDistance>(new QuadDistance()));
    }
}

void CmdLineParser::launchSimulation()
{
    //std::system(SIM_LAUNCH_CMD.c_str());
    bool success = MessageEmulator::getInstance()->start();
    if(!success)
    {
        std::cout << "Error: Failed to start msg emulator." << std::endl;
        exit(0);
    }
}

}
