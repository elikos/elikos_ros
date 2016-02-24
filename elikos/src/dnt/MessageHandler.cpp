#include "MessageHandler.h"

MessageHandler::MessageHandler()
{
    initialize();
}


MessageHandler::~MessageHandler()
{
}


void MessageHandler::initialize()
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("/usb_cam/image_Raw", 1, dispatchMessage);
}


void MessageHandler::dispatchMessage(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat currentImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    //TODO:....
}
