#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>
#include <ros/package.h>

#include "Agent.h"
#include "MessageHandler.h"

namespace ai 
{

const std::string MessageHandler::MAV_FRAME   = "MAV";
const std::string MessageHandler::WORLD_FRAME = "world";
const std::string MessageHandler::TRGT_FRAME  = "trgtRobot";
const std::string MessageHandler::OBS_FRAME   = "obsRobot";

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

}

void MessageHandler::freeInstance()
{
    delete instance_;
}


void MessageHandler::lookupTransform()
{
    lookForMAV();
    lookForObstacles();
    lookForTargets();
    agent_->behave();
}


void MessageHandler::lookForTargets()
{
    for (int i = 0; i < N_TRGT; i++)
    {
        tf::StampedTransform stf;
        try
        {
            listener_.lookupTransform(WORLD_FRAME, TRGT_FRAME + std::to_string(i), ros::Time(0), stf);
            agent_->updateTarget(i, stf.getOrigin(), stf.getRotation());
        }
        catch(tf::TransformException e)
        {
            //TODO: Maybe log exception.
            ROS_ERROR("%s", e.what());
            continue;
        }
    }
}

void MessageHandler::lookForMAV()
{
    tf::StampedTransform stf;
    try
    {
        listener_.lookupTransform(WORLD_FRAME, MAV_FRAME, ros::Time(0), stf);
        agent_->updateQuadRobot(stf.getOrigin(), stf.getRotation());
    }
    catch(tf::TransformException e)
    {
        //TODO: Maybe log exception.
    }
}

void lookForTf()
{
    ros::Rate rate(30);
    while(ros::ok()) 
    {
        lookupTransform();
        ros::spinOnce();
        rate.sleep();
    }
}

void MassageHandler::lookForMessages()
{

}

}
