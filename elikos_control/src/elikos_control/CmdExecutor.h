#ifndef CMD_EXECUTOR
#define CMD_EXECUTOR

#include <memory>

#include "MessageHandler.h"
#include "CmdAbs.h"


class CmdExecutor
{
public:
    CmdExecutor();
    ~CmdExecutor();

    std::unique_ptr<CmdAbs> createCommand(CmdConfig config);  
    void run(); 

private:
    MessageHandler msgHndlr_;

    void checkForNewCommand();

};


#endif //CMD_EXECUTOR