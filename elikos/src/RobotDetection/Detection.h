
#ifndef DETECT_DETECTION_H
#define DETECT_DETECTION_H

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
#include "detect_constants.h"

using namespace std;
using namespace cv;

namespace elikos_detection {

    class Detection {
    public:

        Detection(ros::NodeHandle *nh);
        ~Detection();

        void init();

        void mavPosCallback(const std_msgs::String::ConstPtr &msg);
        void cameraCallback(const sensor_msgs::ImageConstPtr &msg);

        int getXPos();
        int getYPos();

        cv_bridge::CvImagePtr getCurrentImage();
        cv_bridge::CvImagePtr getNextImage();



        void setXPos(int pos);
        void setYPos(int pos);
        void setCapture(VideoCapture capture) {Detection::capture = capture;}

        cv::Scalar getHSVmin();
        cv::Scalar getHSVmax();

        VideoCapture getCapture() const {return capture;}

        void setHSVmin(cv::Scalar scalar);
        void setHSVmax(cv::Scalar scalar);

        void trackFilteredObjects(Mat threshold,Mat HSV, Mat &cameraFeed);

    /* *************************************************************************************************
     * ***              DEBUG FUNCTIONS
     * *************************************************************************************************
     */

        void createTrackbars();
        void setupDebug();
        void captureFrame();
        void trackRobots();
        void on_trackbar(int, void*);


    private:

    /* *************************************************************************************************
     * ***           ROS RELATED FUNCTIONS
     * *************************************************************************************************
     */
        void setPublishers();
        void setSubscribers();

        void removePublishers();
        void removeSubscribers();

    /* *************************************************************************************************
     * ***           ATTRIBUTES
     * *************************************************************************************************
     */

        Mat currentImage;
        cv_bridge::CvImagePtr nextImage;

        int nextPos;
        int currentPos;
        int lasPos;

        int RobotInfo;

        ros::NodeHandle* nh_;

    /* *************************************************************************************************
     * ***              DEBUG ATTRIBUTES
     * *************************************************************************************************
     */

        Mat threshold;
        Mat hsv;
        VideoCapture capture;

    };

}

#endif