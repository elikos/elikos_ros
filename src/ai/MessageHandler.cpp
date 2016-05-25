#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>
#include <ros/package.h>

#include "Agent.h"
#include "MessageHandler.h"

namespace ai 
{

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
}

void MessageHandler::freeInstance()
{
    delete instance_;
    instance_ = nullptr;
}

void MessageHandler::lookForMessages()
{
     
}

}
