#include <string>

#include <ros/ros.h>
#include "CmdOffBoard.h"
#include "CmdLanding.h"
#include "CmdFrontInteraction.h"
#include "CmdTopInteraction.h"
#include "CmdTravel.h"
#include "CmdStandBy.h"

#include "CmdExecutor.h"

CmdExecutor::CmdExecutor()
    : msgHndlr_(&nh_)
{

    commands_.insert(std::pair<int, std::unique_ptr<CmdOffBoard>>(0, std::unique_ptr<CmdOffBoard>(new CmdOffBoard(&nh_, -1))));
    commands_.insert(std::pair<int, std::unique_ptr<CmdLanding>>(1, std::unique_ptr<CmdLanding>(new CmdLanding(&nh_, -1))));
    commands_.insert(std::pair<int, std::unique_ptr<CmdFrontInteraction>>(2, std::unique_ptr<CmdFrontInteraction>(new CmdFrontInteraction(&nh_, -1))));
    commands_.insert(std::pair<int, std::unique_ptr<CmdTopInteraction>>(3, std::unique_ptr<CmdTopInteraction>(new CmdTopInteraction(&nh_, -1))));
    commands_.insert(std::pair<int, std::unique_ptr<CmdTravel>>(4, std::unique_ptr<CmdTravel>(new CmdTravel(&nh_, -1))));
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
        currentCmd_->abort(); // En a-t-on besoin?
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
        if(it->first == 2 || it->first == 3) // Robot interaction
        {
            it->second->setDestination(config.cmdDestination_);
        } 
        if(it->first == 4) // Travel
        {
            it->second->setTrajectory(config.cmdTrajectory_);
        }
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