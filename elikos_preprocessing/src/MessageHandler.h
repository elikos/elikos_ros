//
// Created by olivier on 9/29/16.
//

#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include "PreProcessing.h"

namespace preprocessing {

class MessageHandler
{
public:
    MessageHandler(const ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle);
    ~MessageHandler();
    void lookForMessages();

private:

    void cameraCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
    void imuCallback(const sensor_msgs::Imu& msg);
    void poseCallback(const geometry_msgs::PoseStamped& msg);

    ros::NodeHandle nh_;
    ros::NodeHandle privateNh_;

    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber imageSub_;

    ros::Subscriber imuSub_;
    ros::Subscriber poseSub_;

    PreProcessing preProcessing_;

    image_transport::Publisher preprocessedPub_;
    image_transport::Publisher bwPreprocessedPub_;
};

}

#endif // MESSAGE_HANDLER_H
