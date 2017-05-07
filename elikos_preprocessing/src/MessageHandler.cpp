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

const std::string MessageHandler::IMG_RCV_TOPIC = "/r200_front/image_raw";
const std::string MessageHandler::IMG_RGB_PUB_TOPIC = "/elikos/preprocessed_bw";
const std::string MessageHandler::IMG_BW_PUB_TOPIC = "/elikos/preprocessed_rgb";

MessageHandler* MessageHandler::instance_ = nullptr;

MessageHandler::MessageHandler()
    : it_(nh_)
{
    imageSub_ = it_.subscribe(IMG_RCV_TOPIC, 1, &MessageHandler::cameraCallback, this);
    preprocessedPub_ = it_.advertise(IMG_RGB_PUB_TOPIC, 1);
    bwPreprocessedPub_ = it_.advertise(IMG_BW_PUB_TOPIC, 1);
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
    
    ros::Rate rate(30);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}

void MessageHandler::cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat input = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    cv::Mat output;
    cv::Mat bwOutput;
    preProcessing_.preProcessImage(input, ros::Time::now(), output, bwOutput);

    sensor_msgs::ImagePtr msgPreproc = cv_bridge::CvImage(std_msgs::Header(), "rgb8", output).toImageMsg();
    preprocessedPub_.publish(msgPreproc);

    sensor_msgs::ImagePtr msgPreprocBw = cv_bridge::CvImage(std_msgs::Header(), "mono8", bwOutput).toImageMsg();
    bwPreprocessedPub_.publish(msgPreprocBw);
}

}