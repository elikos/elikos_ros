#include "MessageHandler.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cassert>

MessageHandler::MessageHandler(CalibrationWindow &calibWindow, ControlWindow &controlWindow) : imgTransport_(nh_),
                                                                                               calibWindow_(calibWindow), controlWindow_(controlWindow)
{
    std::string inputTopic, outputTopic, debugInputTopic, commandResultInputTopic;
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

    if (!nh_.getParam("/" + ros::this_node::getName() + "/CommandOutputListenTopic", commandResultInputTopic))
    {
        commandResultInputTopic = "elikos_remotecalib_cmdOutputListenTopic";
    }

    imgSubscriber_ = imgTransport_.subscribe(inputTopic, 1, &MessageHandler::dispatchMessage, this);
    debugimgSubscriber_ = imgTransport_.subscribe(debugInputTopic, 1, &MessageHandler::dispatchDebugImage, this);
    subCommandResult_ = nh_.subscribe(commandResultInputTopic, 100, &MessageHandler::dispatchCommandOutput, this);

    pub_ = nh_.advertise<std_msgs::String>(outputTopic, 10);
    pubImages_ = imgTransport_.advertise(outputTopic + "/debug", 1); //debug only

  
}

MessageHandler::~MessageHandler()
{
}
void MessageHandler::dispatchCommandOutput(const std_msgs::String::ConstPtr &input)
{
    std::stringstream ss(input->data);
    std::string resultCommand; //What command are we getting output for
    ss >> resultCommand;

    if (resultCommand == "getCurrentState")
    {
        int results[27]; //Red[0] Green[9] White[18]
        for (int i = 0; i < 27; ++i)
        {
            ss >> results[i];
        }
        //0= Red, 1=Green, 2=White
        const int selectedColor = controlWindow_.getSelectedColor();
        int *values = controlWindow_.getValues();
        for (int j = 0; j < 9; ++j)
        {
            values[j] = results[j + selectedColor * 9];
        }
        controlWindow_.updateTrackBars();
        calibWindow_.updatePreviewColors();
        // calibWindow_.forceUpdate();
    }
}

void MessageHandler::dispatchDebugImage(const sensor_msgs::ImageConstPtr &input)
{
    cv::Mat currentImage = cv_bridge::toCvCopy(input, IO_IMG_ENCODING_TYPE)->image;

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
    cv::Mat currentImage = cv_bridge::toCvCopy(input, IO_IMG_ENCODING_TYPE)->image;

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