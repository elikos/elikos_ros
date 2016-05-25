#ifndef AI_TRANSLATOR
#define AI_TRANSLATOR

#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <elikos_ros/TargetRobot.h>
#include <elikos_ros/TargetRobotArray.h>

namespace ai
{

class Agent;

class MessageHandler
{
public:
    static const std::string TOPIC;

    static MessageHandler* getInstance();
    static void freeInstance();

    void lookForMessages();

private:
    MessageHandler();
    ~MessageHandler() = default;

    static MessageHandler* instance_;

    ros::NodeHandle nh_;
    Agent* agent_{ nullptr };
    ros::Subscriber sub;
    void handleMessage(const elikos_ros::TargetRobotArray::ConstPtr& input);
};

}

#endif /// MESSAGE_HANDLER
