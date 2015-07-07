
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
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

using namespace std;
using namespace cv;

namespace elikos_detection {

    class Detection {
    public:

        Detection(ros::NodeHandle *nh);
        ~Detection();

        void init();

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
        void cannyEdge();
        void sendMsg();

        string intToString(int number);

        // TF methods
        void initCameraTF();
        void computeTargetPosition();

    /* *************************************************************************************************
     * ***              DEBUG FUNCTIONS
     * *************************************************************************************************
     */

        void createTrackbars();
        void setupDebug();
        void captureFrame();
        void trackBlobs();
        void trackShape();
        void showCurrentImage();
        void showThreshold();


    private:

    /* *************************************************************************************************
     * ***           ROS RELATED FUNCTIONS
     * *************************************************************************************************
     */
        void setPublishers();
        void setSubscribers();

    /* *************************************************************************************************
     * ***           PRIVATE METHODS
     * *************************************************************************************************
     */
        void getRotationFromImage(tf::Quaternion &q);

    /* *************************************************************************************************
     * ***           ATTRIBUTES
     * *************************************************************************************************
     */

        Mat currentImage;
        cv_bridge::CvImagePtr nextImage;
        elikos_ros::RobotsPos robotsPos_msg;
        vector<RobotDesc> vecRobot;
        ros::NodeHandle* nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        ros::Publisher robots_publish;

        // TF transforms
        tf::TransformBroadcaster tf_broadcaster_;
        tf::TransformListener tf_listener_;
        tf::Transform camera_;
        tf::Transform turret_;
        tf::Quaternion turret_rotation_;
        tf::StampedTransform turret_world_;
        tf::Vector3 turret_world_x_;
        tf::Transform target_robot_;


    /* *************************************************************************************************
     * ***              DEBUG ATTRIBUTES
     * *************************************************************************************************
     */

        Mat threshold_w;
        Mat threshold_c;
        Mat hsv;
        Mat cropped_hsv;
        Mat grayscale_image;
        Mat canny;
        Mat contour_drawings;
        VideoCapture capture;

    /* *************************************************************************************************
     * ***               CLASS CONSTANTS
     * *************************************************************************************************
     */

        int H_MIN_W;
        int H_MAX_W;
        int S_MIN_W;
        int S_MAX_W;
        int V_MIN_W;
        int V_MAX_W;
        int H_MIN_C;
        int H_MAX_C;
        int S_MIN_C;
        int S_MAX_C;
        int V_MIN_C;
        int V_MAX_C;
        int PRE_EROSIONS;
        int DILATIONS;
        int POST_EROSIONS;
        int PRE_BLUR;
        int BLUR_AMOUNT;
        int CANNY_THRESH1;
        int CANNY_THRESH2;
        int CANNY_APERTURE;

        const int FRAME_WIDTH = 640;
        const int FRAME_HEIGHT = 480;

        const int MAX_NUM_OBJECTS = 50;

        const int MIN_OBJECT_AREA = 40 * 40;
        const int MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH / 1.5;

        const double CAMERA_FOV_H = 53 * PI/180.0;
        const double CAMERA_FOV_V = 40 * PI/180.0;

        const string windowName = "Original Image";
        const string windowName1 = "HSV Image";
        const string windowName2 = "Thresholded Image";
        const string windowName3 = "After Morphological Operations";
        const string trackbarWindowName = "Trackbars";
        const string shapeDetectTrackbars = "Shape detector";


    };

}

#endif