#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include "MessageHandler.h"

/** pas de main dans un nodelet
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "elikos_preprocessing");

    preprocessing::MessageHandler::getInstance()->lookForMessages();

    return 0;
}
*/

//Exportation de la classe principale du nodelet
PLUGINLIB_EXPORT_CLASS(preprocessing::MessageHandler, nodelet::Nodelet)
