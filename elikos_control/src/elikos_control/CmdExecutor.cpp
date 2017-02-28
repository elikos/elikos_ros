#include <ros/ros.h>
#include "CmdStandBy.h"

#include "CmdExecutor.h"

CmdExecutor::CmdExecutor()
    : msgHndlr_(&nh_)
{
    currentCmd_ = std::unique_ptr<CmdStandBy>(new CmdStandBy(&nh_, -1));            
    pendingCmd_ = std::unique_ptr<CmdStandBy>(new CmdStandBy(&nh_, -1));            
}

CmdExecutor::~CmdExecutor() 
{

}

void CmdExecutor::checkForNewCommand()
{
    CmdConfig lastConfig = msgHndlr_.getLastCmdConfig();
}

std::unique_ptr<CmdAbs> CmdExecutor::createCommand(CmdConfig config)
{
    
}  

void CmdExecutor::run()
{
    ros::Rate r(30);
    while(ros::ok())
    {
        ros::spinOnce(); 

    }
}
