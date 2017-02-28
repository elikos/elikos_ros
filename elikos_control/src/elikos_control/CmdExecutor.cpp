#include <ros/ros.h>
#include "CmdStandBy.h"
#include "CmdOffBoard.h"

#include "CmdExecutor.h"

CmdExecutor::CmdExecutor()
    : msgHndlr_(&nh_)
{
    currentCmd_ = std::unique_ptr<CmdStandBy>(new CmdStandBy(&nh_, -1));            
    pendingCmd_ = std::unique_ptr<CmdStandBy>(new CmdStandBy(&nh_, -1));            
    cmdExecutionThread_ = std::thread(&CmdExecutor::executeCurrentCmd, this);
}

CmdExecutor::~CmdExecutor() 
{

}

void CmdExecutor::checkForNewCommand()
{
    CmdConfig lastConfig = msgHndlr_.getLastCmdConfig();
    if(pendingCmd_->getId() != lastConfig.id_)
    {
        createCommand(lastConfig);
        currentCmd_->abort();
    }
}

std::unique_ptr<CmdAbs> CmdExecutor::createCommand(const CmdConfig& config)
{
    switch(config.cmdCode_)
    {
        case 0:
            pendingCmd_ = std::unique_ptr<CmdOffBoard>(new CmdOffBoard(&nh_, config.id_));
            break;

        case 1:
            break;

        case 2:
            break;

        case 3:
            break;

        case 4:
            break;

        case 5:
            pendingCmd_ = std::unique_ptr<CmdStandBy>(new CmdStandBy(&nh_, config.id_));
            break;

        case 6:
            break;
    } 
}  

void CmdExecutor::executeCurrentCmd()
{
    while (true) 
    {
        currentCmd_->execute();
        currentCmd_.swap(pendingCmd_);
        if (currentCmd_ == nullptr) 
        {
            currentCmd_ = std::unique_ptr<CmdStandBy>(new CmdStandBy(&nh_, -1));
        }
        pendingCmd_ = nullptr;
    }
}

void CmdExecutor::run()
{
    ros::Rate r(30);
    while(ros::ok())
    {
        ros::spinOnce(); 
        checkForNewCommand();
    }
}