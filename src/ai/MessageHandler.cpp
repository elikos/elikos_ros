#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "Agent.h"
#include "MessageHandler.h"

namespace ai 
{

const std::string MessageHandler::TRGT_FRAME     = "trgtRobot";
const std::string MessageHandler::OBS_FRAME      = "obsRobot";
const std::string MessageHandler::MAV_FRAME      = "MAV";
const std::string MessageHandler::WORLD_FRAME    = "world";


MessageHandler::MessageHandler(Agent* agent)
    : agent_(agent)
{
}

void MessageHandler::lookupTransform()
{
}

void MessageHandler::lookForTargets()
{
    for (int i = 0; i < N_TRGT; i++)
    {
        tf::StampedTransform stf;
        try
        {
            std::stringstream frame(TRGT_FRAME);
            frame << std::to_string(i);
            listener_.lookupTransform(WORLD_FRAME, frame.str(), ros::Time(0), stf);
            // TODO: Send to agent.
        }
        catch(tf::TransformException e)
        {
            //TODO: Maybe log exception.
            ROS_ERROR("%s", e.what());
        }
    }
}
void MessageHandler::lookForObs()
{
    for (int i = 0; i < N_OBS; i++)
    {
        tf::StampedTransform stf;
        try
        {
            std::stringstream frame(OBS_FRAME);
            frame << std::to_string(i);
            listener_.lookupTransform(WORLD_FRAME, frame.str(), ros::Time(0), stf);
            // TODO: Send to agent.
        }
        catch(tf::TransformException e)
        {
            //TODO: Maybe log exception.
            ROS_ERROR("%s", e.what());
        }
    }

}
void MessageHandler::lookForMAV()
{
    tf::StampedTransform stf;
    try
    {
        listener_.lookupTransform(WORLD_FRAME, MAV_FRAME, ros::Time(0), stf);
        // TODO: Send to agent.
    }
    catch(tf::TransformException e)
    {
        //TODO: Maybe log exception.
        ROS_ERROR("%s", e.what());
        return;
    }
}

}
