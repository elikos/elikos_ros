#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "Agent.h"
#include "MessageHandler.h"

namespace ai 
{

const static std::string OBS_FRAMES[4];
MessageHandler::MessageHandler(Agent* agent)
    : agent_(agent)
{
}

void MessageHandler::lookupTransform()
{
    tf::StampedTransform stf; 
    try
    {
        listener_.lookupTransform("world", "MAV", ros::Time(0), stf);
    }
    catch(tf::TransformException e)
    {
        ROS_ERROR("%s", e.what());
        ros::Duration(1.0).sleep();
        return;
    }

    std::cout << stf.getOrigin() << std::endl;
    std::cout << stf.getRotation() << std::endl;
}

}
