//
// Created by olivier on 9/29/16.
//

#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#include "CameraInfo.h"

namespace localization {

class QuadState;
class ImageProcessor;

class MessageHandler
{
public:
    MessageHandler(const CameraInfo& cameraInfo, QuadState& state, ImageProcessor* processsor); 
    ~MessageHandler();

    void lookForMessages();

private:

    void cameraCallback(const sensor_msgs::ImageConstPtr& msg);

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber imageSub_;

    tf::TransformListener tfListener_;

    ImageProcessor* const processor_;
    QuadState& state_;

    bool isWaitingForImage_ = false;

    const CameraInfo& cameraInfo_;
};

}

#endif // MESSAGE_HANDLER_H
