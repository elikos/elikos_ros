#ifndef AI_TRANSLATOR
#define AI_TRANSLATOR

#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace ai
{

class Agent;

class MessageHandler
{
public:
    static MessageHandler* getInstance();
    static void freeInstance();

    void lookForMessages();

private:
    MessageHandler();
    ~MessageHandler() = default;

    static MessageHandler* instance_;

    ros::NodeHandle nh_;
    Agent* agent_{ nullptr };
};

}

#endif /// MESSAGE_HANDLER
