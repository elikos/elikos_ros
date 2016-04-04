#ifndef MESSAGE_HANDLER_AI
#define MESSAGE_HANDLER_AI

#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "AIFacade.h"

namespace ai 
{

class MessageHandler
{
public:
    MessageHandler();
    ~MessageHandler();
    void lookupTransform();

private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
};

}



#endif /// MESSAGE_HANDLER
