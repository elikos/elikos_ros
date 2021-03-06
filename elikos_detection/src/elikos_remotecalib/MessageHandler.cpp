#include "MessageHandler.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cassert>

#include "Colors.hpp"

MessageHandler::MessageHandler(CalibrationWindow &calibWindow, ControlWindow &controlWindow) : imgTransport_(nodeHandle_),
                                                                                               calibWindow_(calibWindow), controlWindow_(controlWindow)
{
    std::string inputTopic, outputTopic, debugInputTopic, commandResultInputTopic;
    if (!nodeHandle_.getParam("/" + ros::this_node::getName() + "/topic", inputTopic))
    {
        inputTopic = "";
    }
    if (!nodeHandle_.getParam("/" + ros::this_node::getName() + "/RClistenTopic", debugInputTopic))
    {
        debugInputTopic = "elikos_remotecalib_listenTopic";
    }

    if (!nodeHandle_.getParam("/" + ros::this_node::getName() + "/RCpublishTopic", outputTopic))
    {
        outputTopic = "elikos_remotecalib_publishTopic";
    }

    if (!nodeHandle_.getParam("/" + ros::this_node::getName() + "/CommandOutputListenTopic", commandResultInputTopic))
    {
        commandResultInputTopic = "elikos_remotecalib_cmdOutputListenTopic";
    }

    imgSubscriber_ = imgTransport_.subscribe(inputTopic, 1, &MessageHandler::dispatchMessage, this);
    debugimgSubscriber_ = imgTransport_.subscribe(debugInputTopic, 1, &MessageHandler::dispatchDebugImage, this);
    subCommandResult_ = nodeHandle_.subscribe(commandResultInputTopic, 100, &MessageHandler::dispatchCommandOutput, this);

    pub_ = nodeHandle_.advertise<std_msgs::String>(outputTopic, 10);

}

MessageHandler::~MessageHandler()
{
}
void MessageHandler::dispatchCommandOutput(const std_msgs::String::ConstPtr &input)
{
    std::stringstream inputStream(input->data);
    std::string resultCommand; //What command are we getting output for
    inputStream >> resultCommand;

    if (resultCommand == "getCurrentState")
    {
        static const unsigned COLOR_VALUES_NUMBER = 9;//9 values per color
        static const unsigned RESULTS_LENGHT = NUMBER_OF_COLORS * COLOR_VALUES_NUMBER; 
        int results[RESULTS_LENGHT]; //Red[0] Green[9] White[18]

        for (int i = 0; i < RESULTS_LENGHT; ++i)
        {
            inputStream >> results[i];
        }

        //0= Red, 1=Green, 2=White
        const Color selectedColor = controlWindow_.getSelectedColor();
        int *values = controlWindow_.getValues();

        for (int j = 0; j < COLOR_VALUES_NUMBER; ++j)
        {
            values[j] = results[j + selectedColor * COLOR_VALUES_NUMBER];
        }

        controlWindow_.updateTrackBars();
        calibWindow_.updatePreviewColors();
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
    bool shouldSendCommand = calibWindow_.update(currentImage, outputCommand);
    if (shouldSendCommand)
    {
        std_msgs::String msg; //OutputCommand to be sent
        msg.data = outputCommand;
        pub_.publish(msg);
    }

    //publishing data
}

void MessageHandler::saveCalibration()
{
    std_msgs::String msg; //OutputCommand to be sent
    msg.data = "saveCalib";
    pub_.publish(msg);
}