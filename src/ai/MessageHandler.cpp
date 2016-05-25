#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <ros/package.h>


#include "Agent.h"
#include "MessageHandler.h"

namespace ai 
{
const std::string MessageHandler::TOPIC = "target_robot_array";
MessageHandler* MessageHandler::instance_ = nullptr;

MessageHandler* MessageHandler::getInstance()
{
   if (instance_ == nullptr)
   {
       instance_ = new MessageHandler();
   }
   return  instance_;
}

MessageHandler::MessageHandler()
{
    agent_ = Agent::getInstance();
    sub = nh_.subscribe(TOPIC, 1, &MessageHandler::handleMessage, this);
}

void MessageHandler::freeInstance()
{
    delete instance_;
    instance_ = nullptr;
}

void MessageHandler::lookForMessages()
{
    ros::spin();
}

void MessageHandler::handleMessage(const elikos_ros::TargetRobotArray::ConstPtr& input)
{

}

}
