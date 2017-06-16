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

MessageHandler::MessageHandler(const ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle)
    : nh_(nodeHandle)
    , privateNh_(privateNodeHandle)
    , it_(nh_)
{
    imageSub_ = it_.subscribeCamera(
        "image_rect",
        5,
        &MessageHandler::cameraCallback,
        this,
        image_transport::TransportHints("raw", ros::TransportHints(), privateNh_)
    );
    preprocessedPub_ = it_.advertise("image_preprocessed", 1);
}

MessageHandler::~MessageHandler()
{
}

void MessageHandler::cameraCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    cv::Mat input = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
    cv::Mat output;
    cv::Mat bwOutput;
    preProcessing_.setFocalLength(info_msg->K[0]);//TODO avoir des longeurs focales différentes pour x et y
    preProcessing_.preProcessImage(input, ros::Time::now(), output, bwOutput);

    sensor_msgs::ImagePtr msgPreproc = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output).toImageMsg();
    preprocessedPub_.publish(msgPreproc);
}

}