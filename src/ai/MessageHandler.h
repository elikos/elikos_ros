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
    static const int N_TRGT = 10;
    static const int N_OBS = 4;

    static const std::string TRGT_FRAME;
    static const std::string OBS_FRAME;
    static const std::string MAV_FRAME;
    static const std::string WORLD_FRAME;

    MessageHandler(Agent* agent);
    ~MessageHandler() = default;
    MessageHandler() = default;

    inline void setAgent(Agent* agent);
    void lookupTransform();

private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    Agent* agent_{ nullptr };

    void lookForTargets();
    void lookForObstacles();
    void lookForMAV();
};


inline void MessageHandler::setAgent(Agent* agent)
{
    agent_ = agent;
}

}

#endif /// MESSAGE_HANDLER
