#include "MessageHandler.h"

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "BlobDetection/BlobTracking.h"

MessageHandler::MessageHandler() : 
    it_(nh_)
{
}


MessageHandler::~MessageHandler()
{
    is_ = it_.subscribe("/usb_cam/image_Raw", 1, dispatchMessage);
}


void MessageHandler::dispatchMessage(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat currentImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

    Mat threshold_w;
    Mat threshold_r;
    Mat threshold_g;
    Mat robotsMat;

    BlobTracking tracking;

    tracking.track(currentImage, threshold_w, threshold_r, threshold_g, robotsMat);
}


