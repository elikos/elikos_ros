#include <sstream>
#include <string>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp">
#include "RobotDesc.h"
#include <ros/ros.h>
#include <sensor_msgs.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "./../defines.cpp"

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

    if(calibrationMode)
    {
        detect_instance.createTrackbars();
    }

    detect_instance.setupDebug();

    while(ros::ok())
    {
        detect_instance.captureFrame();
        if(calibrationMode == true)
        {
            detect_instance.trackRobots();
        }



        waitkey(30);
    }

    return 0;
}