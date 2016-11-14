#include "MessageHandler.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

MessageHandler::MessageHandler(CalibrationWindow &calibWindow, ControlWindow &controlWindow) : imgTransport_(nh_), 
calibWindow_(calibWindow), controlWindow_(controlWindow)
{
    std::string inputTopic, outputTopic, debugInputTopic;
    if (!nh_.getParam("/" + ros::this_node::getName() + "/topic", inputTopic))
    {
        inputTopic = "";
    }
    if (!nh_.getParam("/" + ros::this_node::getName() + "/RClistenTopic", debugInputTopic))
    {
        debugInputTopic = "elikos_remotecalib_listenTopic";
    }

    if (!nh_.getParam("/" + ros::this_node::getName() + "/RCpublishTopic", outputTopic))
    {
        outputTopic = "elikos_remotecalib_publishTopic";
    }

    imgSubscriber_ = imgTransport_.subscribe(inputTopic, 1, &MessageHandler::dispatchMessage, this);
    debugimgSubscriber_ = imgTransport_.subscribe(debugInputTopic, 1, &MessageHandler::dispatchDebugImage, this);

    pub_ = nh_.advertise<std_msgs::String>(outputTopic, 1);
    pubImages_ = imgTransport_.advertise(outputTopic + "/debug", 1); //debug only
}

MessageHandler::~MessageHandler()
{
}

void MessageHandler::dispatchDebugImage(const sensor_msgs::ImageConstPtr &input)
{
    cv::Mat currentImage = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8)->image;

    // cv::Mat outputImage;
    std::string outputCommand;

    //DO DATA PARSING AND COMMAND OUTPUT
    if (controlWindow_.update(currentImage, outputCommand))
    {
        std_msgs::String msg; //OutputCommand to be sent
        msg.data = outputCommand;
        pub_.publish(msg);
    }
}
void MessageHandler::dispatchMessage(const sensor_msgs::ImageConstPtr &input)
{
    cv::Mat currentImage = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8)->image;

    // cv::Mat outputImage;
    std::string outputCommand;

    //DO DATA PARSING AND COMMAND OUTPUT
    if (calibWindow_.update(currentImage, outputCommand))
    {
        std_msgs::String msg; //OutputCommand to be sent
        msg.data = outputCommand;
        pub_.publish(msg);
    }

    //debug images
    // sensor_msgs::ImagePtr msgDebug = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outputImage).toImageMsg();
    // pubImages_.publish(msgDebug);
    //publishing data
}

void MessageHandler::saveCalibration()
{
    std_msgs::String msg; //OutputCommand to be sent
    msg.data = "saveCalib";
    pub_.publish(msg);
}