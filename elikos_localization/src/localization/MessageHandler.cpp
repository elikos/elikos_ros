//
// Created by olivier on 9/29/16.
//

#include "MessageHandler.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
#include "ImageProcessor.h"

namespace localization {

MessageHandler* MessageHandler::instance_ = nullptr;

MessageHandler::MessageHandler()
    : it_(nh_)
{
    //imageSub_ = it_.subscribe("/camera/image_raw", 1, &MessageHandler::cameraCallback, this);
    imuSub_ = nh_.subscribe("/mavros/imu/data", 1, &MessageHandler::imuCallback, this);
}

MessageHandler::~MessageHandler()
{
}

MessageHandler* MessageHandler::getInstance()
{
    if (instance_ == nullptr) {
        instance_ = new MessageHandler();
    }
    return instance_;
}

void MessageHandler::freeInstance()
{
    delete instance_;
}

void MessageHandler::lookForMessages()
{
    cv::VideoCapture vc;
    vc.open("/home/olivier/Videos/cam_gauche.mp4");
    if (!vc.isOpened())
    {
        exit(-1);
    }
    ros::Rate rate(30);
    while(ros::ok())
    {
        cv::Mat frame;
        vc >> frame;
        ImageProcessor::getInstance()->processImage(frame);

        ros::spinOnce();
        rate.sleep();
    }
}

void MessageHandler::cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat input = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8)->image;
    ImageProcessor::getInstance()->processImage(input);
}

void MessageHandler::imuCallback(const sensor_msgs::ImuConstPtr msg)
{
    tf::Quaternion q;
    tf::Vector3 v;
    tf::vector3MsgToTF(msg->linear_acceleration, v);
    v.normalize();

    ImageProcessor::getInstance()->theta_ = atan(v.z() / -v.x());
}

}