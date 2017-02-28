#ifndef CMD_EXECUTOR_H
#define CMD_EXECUTOR_H

#include <memory>

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

    void checkForNewCommand();
    std::unique_ptr<CmdAbs> createCommand(CmdConfig config);  
};

#endif // CMD_EXECUTOR_H