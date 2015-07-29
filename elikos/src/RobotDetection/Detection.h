
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
#include <sensor_msgs/Range.h>
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
        void altitudeCallback(const sensor_msgs::RangeConstPtr& msg);

        Mat getCurrentImage(){return currentImage;}
        cv_bridge::CvImagePtr getNextImage(){return nextImage;}

        void setCurrentImage(Mat image){currentImage = image;}
        void setNextImage(cv_bridge::CvImagePtr image){nextImage = image;}

        void setCapture(VideoCapture capture) {Detection::capture = capture;}

        VideoCapture getCapture(){return capture;}

        vector<RobotDesc> trackFilteredObjects(Mat threshold, Mat &cameraFeed);
        void drawObjects(vector<RobotDesc> vecRobot, Mat &frame);
        void drawObject(RobotDesc robot, Mat &frame);
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
    void getRotationFromImage(tf::Quaternion &q, int i);
    void getRotationFromImage(tf::Quaternion &q, RobotDesc desc);

    /* *************************************************************************************************
     * ***           ATTRIBUTES
     * *************************************************************************************************
     */

        Mat currentImage;
        cv_bridge::CvImagePtr nextImage;
        elikos_ros::RobotsPos robotsPos_msg;
        vector<RobotDesc> foundRobots;
        ros::NodeHandle* nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;

        ros::Subscriber sub;
        float altRange;

        ros::Publisher robots_publish;
        double min_distance = -1;

        // TF transforms
        tf::TransformBroadcaster tf_broadcaster_;
        tf::TransformListener tf_listener_;
        tf::Transform camera_;
        std::vector<std::string> turret_frame_id_/*{"turret0",
                                                  "turret1",
                                                  "turret2",
                                                  "turret3",
                                                  "turret4",
                                                  "turret5" }*/;
        std::vector<tf::Transform> turret_;
        std::vector<tf::Quaternion> turret_rotation_;
        std::vector<tf::StampedTransform> turret_world_;
        std::vector<tf::Vector3> turret_world_x_;
        tf::Transform target_robot_;


    /* *************************************************************************************************
     * ***              DEBUG ATTRIBUTES
     * *************************************************************************************************
     */

        Mat threshold_w;
        Mat threshold_g;
        Mat threshold_r;
        Mat hsv_w;
        Mat hsv_c;
        Mat mask;
        Mat closeWhite;
        Mat cropped_hsv;
        Mat grayscale_image;
        Mat canny;
        Mat morph_ex;
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
        int H_MIN_G;
        int H_MAX_G;
        int S_MIN_G;
        int S_MAX_G;
        int V_MIN_G;
        int V_MAX_G;
        int H_MIN_R;
        int H_MAX_R;
        int S_MIN_R;
        int S_MAX_R;
        int V_MIN_R;
        int V_MAX_R;
        int PRE_EROSIONS_W;
        int DILATIONS_W;
        int POST_EROSIONS_W;
        int PRE_EROSIONS_G;
        int DILATIONS_G;
        int POST_EROSIONS_G;
        int PRE_BLUR;
        int BLUR_AMOUNT;
        int CANNY_THRESH1;
        int CANNY_THRESH2;
        int CANNY_APERTURE;
        int POLY_AREA_MIN;
        int POLY_AREA_MAX;
        int MORPH_OP;
        int MORPH_ELEMENT;
        int MORPH_SIZE;
        int MAX_DIST;

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
        const string HSVTrackbars= "HSV Trackbars";
        const string shapeDetectTrackbars = "Shape detector";


    };

}

#endif