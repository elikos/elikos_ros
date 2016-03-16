#ifndef MESSAGE_HANDLER
#define MESSAGE_HANDLER

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class MessageHandler
{
public:
    MessageHandler();
    ~MessageHandler();
    static void dispatchMessage(const sensor_msgs::ImageConstPtr &msg);

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber is_;
};

#endif /// MESSAGE_HANDLER

