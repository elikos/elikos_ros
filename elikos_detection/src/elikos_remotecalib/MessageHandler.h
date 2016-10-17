#ifndef MESSAGE_HANDLER_REMOTECALIB
#define MESSAGE_HANDLER_REMOTECALIB
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "gui/WindowCV.hpp"

class MessageHandler
{
public:
    MessageHandler(WindowCV& calibWindow);
    ~MessageHandler();
    void dispatchMessage(const sensor_msgs::ImageConstPtr &input);
    void saveCalibration();

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber is_;
    ros::Publisher pub_;
    WindowCV& calibWindow_;

    image_transport::Publisher pubImages_;//debug only
};

#endif /// MESSAGE_HANDLER_TTF
