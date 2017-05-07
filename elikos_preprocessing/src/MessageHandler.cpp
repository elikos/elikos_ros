//
// Created by olivier on 9/29/16.
//

#include "MessageHandler.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>

#include "PreProcessing.h"

namespace preprocessing {

MessageHandler* MessageHandler::instance_ = nullptr;

MessageHandler::MessageHandler()
    : it_(nh_)
{
    imageSub_ = it_.subscribe("/camera/image_raw", 1, &MessageHandler::cameraCallback, this);
    preprocessedPub_ = it_.advertise("elikos/preprocessed", 1);
    bwPreprocessedPub_ = it_.advertise("elikos/preprocessed_bw", 1);
    preProcessing_ = new PreProcessing();
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

void MessageHandler::initMsgHandler(){
    preProcessing_ = new PreProcessing();
}

void MessageHandler::freeInstance()
{
    delete instance_;
}

void MessageHandler::lookForMessages()
{
    
    ros::Rate rate(30);
    while(ros::ok())
    {
        //cv::Mat frame;
        //vc >> frame;
        //ImageProcessor::getInstance()->processImage(frame, ros::Time::now());

        ros::spinOnce();
        rate.sleep();
    }
}

void MessageHandler::cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat input = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    cv::Mat output;
    cv::Mat bwOutput;
    preProcessing_->preProcessImage(input, output, bwOutput);

    sensor_msgs::ImagePtr msgPreproc = cv_bridge::CvImage(std_msgs::Header(), "rgb8", output).toImageMsg();
    preprocessedPub_.publish(msgPreproc);

    sensor_msgs::ImagePtr msgPreprocBw = cv_bridge::CvImage(std_msgs::Header(), "mono8", bwOutput).toImageMsg();
    bwPreprocessedPub_.publish(msgPreprocBw);
    
}

}