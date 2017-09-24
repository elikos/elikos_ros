//
// Created by olivier on 9/29/16.
//

#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <elikos_main/StampedMatrix3.h>
#include "PreProcessing.h"

#include "CameraInfo.h"

namespace preprocessing {

class MessageHandler
{
public:
    MessageHandler(const ros::NodeHandle& nodeHandle, const std::string& cameraName);
    ~MessageHandler();

private:

    void cameraCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg);

    CameraInfo cameraInfo_;
    QuadState state_;

    ros::NodeHandle nh_;

    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber imageSub_;
    image_transport::Publisher preprocessedPub_;
    ros::Publisher inverseTransformPub_;

    PreProcessing preProcessing_;

};

}

#endif // MESSAGE_HANDLER_H
