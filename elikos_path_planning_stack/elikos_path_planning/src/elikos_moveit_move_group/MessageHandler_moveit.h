#ifndef MESSAGE_HANDLER_moveit
#define MESSAGE_HANDLER_moveit


#include "moveit_move_group.h"

class MessageHandler_moveit
{
public:
    MessageHandler_moveit();
    ~MessageHandler_moveit();
    void dispatchMessageTarget(const elikos_main::AICmd::ConstPtr &input);
    elikos_main::AICmd getAICmd();
    bool hasNewMessage();
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    elikos_main::AICmd input_;
    bool hasNewMessage_;
};

#endif /// MESSAGE_HANDLER_TTF
