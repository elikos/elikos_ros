#ifndef MESSAGE_HANDLER
#define MESSAGE_HANDLER

#include <ros/ros.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class MessageHandler
{
public:
    MessageHandler();
    ~MessageHandler();
    void initialize();
    static void dispatchMessage(const sensor_msgs::ImageConstPtr &msg);
};

#endif /// MESSAGE_HANDLER

