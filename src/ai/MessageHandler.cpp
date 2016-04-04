#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "AIFacade.h"
#include "MessageHandler.h"

namespace ai 
{

MessageHandler::MessageHandler()
{

}


MessageHandler::~MessageHandler()
{

}


void MessageHandler::lookupTransform()
{
    tf::StampedTransform stf; 
    listener_.lookupTransform("", "", ros::Time(0), stf);
}

}
