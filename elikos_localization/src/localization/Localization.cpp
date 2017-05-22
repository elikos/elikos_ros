//
// Created by olivier on 9/29/16.
//

#include <ros/ros.h>

#include "QuadState.h"
#include "ImageProcessor.h"
#include "MessageHandler.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "elikos_localization");

    localization::QuadState state;
    localization::ImageProcessor processor(&state);
    localization::MessageHandler msgHdl(&state, &processor);

    msgHdl.lookForMessages();

    return 0;
}
