#ifndef CMD_EXECUTOR_H
#define CMD_EXECUTOR_H

#include <memory>
#include <thread>

#include "MessageHandler.h"
#include "CmdAbs.h"

class CmdExecutor
{
public:
    CmdExecutor();
    ~CmdExecutor();

    void run(); 

private:
    ros::NodeHandle nh_;
    MessageHandler msgHndlr_;

    std::unique_ptr<CmdAbs> pendingCmd_;
    std::unique_ptr<CmdAbs> currentCmd_;

    void executeCurrentCmd();
    void checkForNewCommand();

    std::thread cmdExecutionThread_;
    std::unique_ptr<CmdAbs> createCommand(const CmdConfig& config);  
};

#endif // CMD_EXECUTOR_H