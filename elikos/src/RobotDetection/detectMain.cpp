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

//#define DEBUG_DETECT 1
//#define USE_WEBCAM 1

using namespace cv;

int main(int argc, char* argv[])
{
    //init ROS
    ros::init( argc, argv, "elikos_robotdetect" );
    ros::NodeHandle nh;
    ros::Rate r(30);

    // Load parameters
    bool DEBUG_MODE, USE_WEBCAM;
    nh.param<bool>("debug_mode", DEBUG_MODE, true);
    nh.param<bool>("use_webcam", USE_WEBCAM, false);

    //init Detection class : set up subs/pubs
    elikos_detection::Detection detect_instance(&nh);
    detect_instance.init();

    if (true) {
        detect_instance.createTrackbars();
    }

    if (USE_WEBCAM) {
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

        if (!detect_instance.getCurrentImage().empty()) {

            detect_instance.trackBlobs();
            //detect_instance.trackShape();

            if (true) {
                detect_instance.showThreshold();
                detect_instance.showCurrentImage();
            }
        }

        detect_instance.computeTargetPosition();

        waitKey(30);
    }

    return 0;
}
