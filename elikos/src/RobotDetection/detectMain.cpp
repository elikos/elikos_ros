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
#define USE_WEBCAM 1

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
        if (USE_WEBCAM) {
            detect_instance.captureFrame();
        }

        else {
            ros::spinOnce();
        }

        if (1){//detect_instance.getCurrentImage().dims != 0) {

            detect_instance.trackRobots();
            detect_instance.sendMsg();

            if (DEBUG_DETECT) {
                detect_instance.showThreshold();
                detect_instance.showCurrentImage();
            }
        }

        waitKey(30);
    }

    return 0;
}