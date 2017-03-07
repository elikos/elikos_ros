#include <string>

#include <ros/ros.h>
#include "CmdStandBy.h"
#include "CmdOffBoard.h"

#include "CmdExecutor.h"

CmdExecutor::CmdExecutor()
    : msgHndlr_(&nh_)
{

    commands_.insert(std::pair<int, std::unique_ptr<CmdOffBoard>>(0, std::unique_ptr<CmdOffBoard>(new CmdOffBoard(&nh_, -1))));
    commands_.insert(std::pair<int, std::unique_ptr<CmdStandBy>>(5, std::unique_ptr<CmdStandBy>(new CmdStandBy(&nh_, -1))));
    currentCmd_ = commands_[5].get();
    pendingCmd_ = currentCmd_;

    cmdExecutionThread_ = std::thread(&CmdExecutor::executeCurrentCmd, this);
}

CmdExecutor::~CmdExecutor() 
{

}

void CmdExecutor::checkForNewCommand()
{
    CmdConfig lastConfig = msgHndlr_.getLastCmdConfig();
    pendingCmdLock_.lock();
    int pendingId = pendingCmd_->getId();
    if(pendingId != lastConfig.id_)
    {
        createCommand(lastConfig);
        currentCmd_->abort();
    }
    pendingCmdLock_.unlock();
}

void CmdExecutor::createCommand(const CmdConfig& config)
{

    std::unordered_map<int, std::unique_ptr<CmdAbs>>::iterator it;
    it = commands_.find(config.cmdCode_);
    if (it != commands_.end())
    {
        pendingCmd_ = it->second.get();
        it->second->setId(config.id_); 
    }
}  

void CmdExecutor::executeCurrentCmd()
{
    while (true) 
    {
        currentCmd_->execute();
        pendingCmdLock_.lock();
        currentCmd_ = pendingCmd_;
        pendingCmd_ = commands_[5].get();
        pendingCmdLock_.unlock();
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