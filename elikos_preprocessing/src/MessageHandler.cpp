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


MessageHandler::MessageHandler()
    : it_(nh_)
{
    std::string nodeNamespace = ros::this_node::getName();
    bool hasParams = nh_.getParam(nodeNamespace + "/topic", IMG_RCV_TOPIC) &&
                     nh_.getParam(nodeNamespace + "/preprocessed_topic", IMG_PUB_TOPIC);

    if (!hasParams) {
        ROS_FATAL("Could not find expteced parameters: topic, preprocessed_topic");
        exit(1);
    }

    imageSub_ = it_.subscribe(IMG_RCV_TOPIC, 1, &MessageHandler::cameraCallback, this);
    preprocessedPub_ = it_.advertise(IMG_PUB_TOPIC, 1);
}

MessageHandler::~MessageHandler()
{
}

MessageHandler* MessageHandler::getInstance()
{
    static MessageHandler instance_;
    return &instance_;
}

void MessageHandler::lookForMessages()
{
    ros::spin();
}

void MessageHandler::cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat input = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    cv::Mat output;
    cv::Mat bwOutput;
    preProcessing_.preProcessImage(input, ros::Time::now(), output, bwOutput);

    sensor_msgs::ImagePtr msgPreproc = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output).toImageMsg();
    preprocessedPub_.publish(msgPreproc);
}

}