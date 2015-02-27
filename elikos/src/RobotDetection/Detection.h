
#ifndef DETECT_DETECTION_H
#define DETECT_DETECTION_H

#include <sstream>
#include <string>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "std_msgs/String.h"
#include "RobotDesc.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "./../defines.cpp"
#include <sensor_msgs/image_encodings.h>
#include <elikos_ros/RobotPos.h>
#include <elikos_ros/RobotsPos.h>

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

        Mat getCurrentImage(){return currentImage;}
        cv_bridge::CvImagePtr getNextImage(){return nextImage;}

        void setCurrentImage(Mat image){currentImage = image;}
        void setNextImage(cv_bridge::CvImagePtr image){nextImage = image;}

        void setCapture(VideoCapture capture) {Detection::capture = capture;}

        VideoCapture getCapture(){return capture;}

        void trackFilteredObjects(Mat threshold,Mat HSV, Mat &cameraFeed);
        void drawObject(vector<RobotDesc> vecRobot,Mat &frame);
        void morphOps(Mat &thresh);

        string intToString(int number);

    /* *************************************************************************************************
     * ***              DEBUG FUNCTIONS
     * *************************************************************************************************
     */

        void createTrackbars();
        void setupDebug();
        void captureFrame();
        void trackRobots();
        void showCurrentImage();


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

        elikos_ros::RobotsPos robotsPos_msg;

        ros::NodeHandle* nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        ros::Publisher robots_publish;

    /* *************************************************************************************************
     * ***              DEBUG ATTRIBUTES
     * *************************************************************************************************
     */

        Mat threshold;
        Mat hsv;
        VideoCapture capture;

    /* *************************************************************************************************
     * ***               CLASS CONSTANTS
     * *************************************************************************************************
     */

        int H_MIN;
        int H_MAX;
        int S_MIN;
        int S_MAX;
        int V_MIN;
        int V_MAX;

        const int FRAME_WIDTH = 640;
        const int FRAME_HEIGHT = 480;

        const int MAX_NUM_OBJECTS = 50;

        const int MIN_OBJECT_AREA = 40 * 40;
        const int MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH / 1.5;

        const string windowName = "Original Image";
        const string windowName1 = "HSV Image";
        const string windowName2 = "Thresholded Image";
        const string windowName3 = "After Morphological Operations";
        const string trackbarWindowName = "Trackbars";


    };

}

#endif