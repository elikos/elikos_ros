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

class QuadState;

namespace localization {

class ImageProcessor;

class MessageHandler
{
public:
    MessageHandler(const CameraInfo& cameraInfo, QuadState& state, ImageProcessor* processsor); 
    ~MessageHandler();

    void lookForMessages();

private:

    void cameraCallback(const sensor_msgs::ImageConstPtr& msg);

    bool isProcessing_ = false;

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    image_transport::Subscriber imageSub_;
    image_transport::Publisher imagePub_;

    ImageProcessor* const processor_;
    QuadState& state_;

    const CameraInfo& cameraInfo_;
};

}

#endif // MESSAGE_HANDLER_H
