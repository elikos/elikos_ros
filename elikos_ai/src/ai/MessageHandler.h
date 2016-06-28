#ifndef AI_MESSAGE_HANDLER_H
#define AI_MESSAGE_HANDLER_H

#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <elikos_ros/TargetRobot.h>
#include <elikos_ros/TargetRobotArray.h>
#include <tf/transform_broadcaster.h>

namespace tf
{
    class Vector3;
}

namespace ai
{

class Agent;

class MessageHandler
{
public:
    static const std::string TRGT_TOPIC;
    static const std::string SETPOINT_TOPIC;
    static const std::string MAV_TOPIC;
    static const std::string WORLD_FRAME;

    static MessageHandler* getInstance();
    static void freeInstance();

    void lookForMessages();
    void sendDestination(const tf::Vector3& destination);

private:
    static MessageHandler* instance_;

    ros::NodeHandle nh_;
    ros::Subscriber trgtSub_;
    ros::Subscriber mavSub_;
    ros::Publisher mavPub_;
    tf::TransformBroadcaster br_;

    Agent* agent_{ nullptr };

    void handleTrgtMsg(const elikos_ros::TargetRobotArray::ConstPtr& input);
    void handleMavMsg(const geometry_msgs::PoseStamped::ConstPtr& input);

    MessageHandler();
    ~MessageHandler() = default;
};

}

#endif /// MESSAGE_HANDLER
