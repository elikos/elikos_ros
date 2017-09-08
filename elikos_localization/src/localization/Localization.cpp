//
// Created by olivier on 9/29/16.
//

#include <ros/ros.h>

#include "CameraInfo.h"

#include "QuadState.h"
#include "ImageProcessor.h"
#include "MessageHandler.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "elikos_localization");

    CameraInfo cameraInfo;
    std::string camera_name;
    bool initSucceed = argc > 1;
    if (argc > 1) 
    {
        camera_name = std::string(argv[1]);
        cameraInfo.load(camera_name);
    }

    QuadState state(cameraInfo.frame);
    localization::ImageProcessor processor(cameraInfo, state);
    localization::MessageHandler msgHdl(cameraInfo, state, &processor);

    msgHdl.lookForMessages();

    return 0;
}
