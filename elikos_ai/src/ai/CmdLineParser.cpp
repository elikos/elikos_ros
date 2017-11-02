#include <cstring>
#include <ros/ros.h>
#include <thread>

#include "CmdLineParser.h"

#include "Agent.h"

namespace ai
{

const char* CmdLineParser::SIM_LAUNCH_CMD{ "roslaunch elikos_sim simulation.launch" };
const char* CmdLineParser::ARG_MODE { "--mode" };

CmdLineParser::CmdLineParser(int argc, char* argv[])
    : argc_(argc), argv_(argv)
{
}


void CmdLineParser::parse()
{
    int mode = checkModeArg();
    if (mode > 0)
    {
        parseMode(mode);
    }
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
    if ((mode & TARGET_DESTINATION_MASK) == TARGET_DESTINATION_MASK)
    {
        Agent::getInstance()->addConsideration(Agent::Consideration::TARGET_DESTINATION);
    }
    if ((mode & QUAD_DISTANCE_MASK) == QUAD_DISTANCE_MASK)
    {
        Agent::getInstance()->addConsideration(Agent::Consideration::QUAD_DISTANCE);
    }
    if ((mode & CLUSTER_SIZE_MASK) == CLUSTER_SIZE_MASK)
    {
        Agent::getInstance()->addConsideration(Agent::Consideration::CLUSTER_SIZE);
    }
}

}