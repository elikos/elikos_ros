//
// Created by olivier on 9/29/16.
//

#include "MessageHandler.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ImageProcessor.h"


namespace localization {

MessageHandler* MessageHandler::instance_ = nullptr;

MessageHandler::MessageHandler()
    : it_(nh_)
{
    //imageSub_ = it_.subscribe("/camera/image_raw", 1, &MessageHandler::CameraCallback, this);
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
    vc.open("/home/olivier/Videos/cam_bas.mp4");
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

void MessageHandler::CameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat input = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8)->image;
    ImageProcessor::getInstance()->processImage(input);
}

}