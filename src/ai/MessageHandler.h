#ifndef MESSAGE_HANDLER_AI
#define MESSAGE_HANDLER_AI

#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace ai
{

class Agent;

class MessageHandler
{
public:
    MessageHandler() = default;
    MessageHandler(Agent* agent);
    ~MessageHandler() = default;

    inline void setAgent(Agent* agent);
    void lookupTransform();

private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    Agent* agent_{ nullptr };
};


inline void MessageHandler::setAgent(Agent* agent)
{
    agent_ = agent;
}

}

#endif /// MESSAGE_HANDLER
