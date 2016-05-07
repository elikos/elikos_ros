#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "Agent.h"
#include "MessageHandler.h"

namespace ai 
{

MessageHandler::MessageHandler(Agent* agent)
    : agent_(agent)
{
}

void MessageHandler::lookupTransform()
{
    tf::StampedTransform stf; 
    listener_.lookupTransform("world", "MAV", ros::Time(0), stf);

    stf.getOrigin();
    stf.getRotation();
}

}
