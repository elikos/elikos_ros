#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "Agent.h"
#include "MessageHandler.h"

namespace ai 
{

const std::string MessageHandler::MAV_FRAME   = "MAV";
const std::string MessageHandler::WORLD_FRAME = "world";
const std::string MessageHandler::TRGT_FRAME  = "trgtRobot";
const std::string MessageHandler::OBS_FRAME   = "obsRobot";


MessageHandler::MessageHandler(Agent* agent)
    : agent_(agent)
{
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

void MessageHandler::lookForObstacles()
{
    for (int i = 0; i < N_OBS; i++)
    {
        tf::StampedTransform stf;
        try
        {
            listener_.lookupTransform(WORLD_FRAME, OBS_FRAME + std::to_string(i), ros::Time(0), stf);
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

}
