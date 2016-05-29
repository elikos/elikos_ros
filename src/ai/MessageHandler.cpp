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
    std::cout << "message" << std::endl;
    agent_ = Agent::getInstance();
    sub = nh_.subscribe<elikos_ros::TargetRobotArray>(TOPIC, 1, &MessageHandler::handleMessage, this);
}

void MessageHandler::freeInstance()
{
    delete instance_;
    instance_ = nullptr;
}

void MessageHandler::lookForMessages()
{
    std::cout << "message" << std::endl;
    ros::spin();
}

void MessageHandler::handleMessage(const elikos_ros::TargetRobotArray::ConstPtr& input)
{
    std::cout << "message" << std::endl;
    unsigned long size = input->targets.size();
    std::cout << size << std::endl;
}

}
