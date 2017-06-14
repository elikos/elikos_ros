#include <ros/ros.h>

#include "MessageHandler.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "elikos_preprocessing");

    preprocessing::MessageHandler::getInstance()->lookForMessages();

    return 0;
}
