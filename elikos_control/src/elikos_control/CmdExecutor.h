#ifndef CMD_EXECUTOR_H
#define CMD_EXECUTOR_H

#include <memory>
#include <thread>
#include <mutex>

#include <unordered_map>

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

    CmdAbs* pendingCmd_;
    CmdAbs* currentCmd_;

    void executeCurrentCmd();
    void checkForNewCommand();

    std::unordered_map<int, std::unique_ptr<CmdAbs>> commands_;

    std::mutex pendingCmdLock_;
    std::thread cmdExecutionThread_;
    void createCommand(const CmdConfig& config);  
};

#endif // CMD_EXECUTOR_H