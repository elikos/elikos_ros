//
// Created by olivier on 9/29/16.
//

#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

namespace localization {

class QuadState;
class ImageProcessor;

class MessageHandler
{
public:
    MessageHandler(QuadState* state, ImageProcessor* processsor); 
    ~MessageHandler();

    void lookForMessages();

private:

    void cameraCallback(const sensor_msgs::ImageConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu& msg);
    void poseCallback(const geometry_msgs::PoseStamped& msg);

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber imageSub_;
    ros::Subscriber imuSub_;
    ros::Subscriber poseSub_;

    ImageProcessor* const processor_;
    QuadState* const state_;
};

}

#endif // MESSAGE_HANDLER_H
