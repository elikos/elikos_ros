#ifndef CMD_EXECUTOR_H
#define CMD_EXECUTOR_H

#include <memory>
#include <thread>
#include <mutex>

#include <unordered_map>

#include "MessageHandler.h"
#include "CmdAbs.h"


enum OrderToGive {
        ABORT,
        AJUST,
        CONTINUE
    };

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

    trajectory_msgs::MultiDOFJointTrajectory pendingTrajectory_;
    geometry_msgs::Pose pendingDestination_;

    void executeCurrentCmd();
    void checkForNewCommand();
    OrderToGive checkNextOrder();

    std::unordered_map<int, std::unique_ptr<CmdAbs>> commands_;

    std::mutex pendingCmdLock_;
    std::thread cmdExecutionThread_;
    void createCommand(const CmdConfig& config);
};

#endif // CMD_EXECUTOR_H