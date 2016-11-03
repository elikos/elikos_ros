//
// Created by olivier on 9/29/16.
//

#include <ros/ros.h>

#include "MessageHandler.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "elikos_localization");

    localization::MessageHandler::getInstance()->lookForMessages();

    return 0;
}
