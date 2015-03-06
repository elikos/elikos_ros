#include <sstream>
#include <string>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "RobotDesc.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "./../defines.cpp"
#include "Detection.h"
#include "std_msgs/String.h"

#define DEBUG_DETECT 1

using namespace cv;

int main(int argc, char* argv[])
{
    //init ROS

    ros::init( argc, argv, "elikos_robotdetect" );
    ros::NodeHandle nh;
    ros::Rate r(30);

    //init Detection class : set up subs/pubs
    elikos_detection::Detection detect_instance(&nh);
    detect_instance.init();

    bool calibrationMode = true ;

    if(DEBUG_DETECT) {
        detect_instance.createTrackbars();
        detect_instance.setupDebug();
    }

    while(ros::ok())
    {

        if(DEBUG_DETECT)
            detect_instance.captureFrame();
        else
            detect_instance.setCurrentImage(detect_instance.getNextImage()->image);

        detect_instance.trackRobots();

        if(DEBUG_DETECT)
            detect_instance.showCurrentImage();

        ros::spinOnce();
        waitKey(30);
    }

    return 0;
}