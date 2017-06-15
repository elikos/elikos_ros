//
// Created by olivier on 9/29/16.
//

#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include "PreProcessing.h"

namespace preprocessing {

class MessageHandler
{
public:
    static MessageHandler* getInstance();
    static void freeInstance();

    void lookForMessages();

private:
    
    MessageHandler();
    ~MessageHandler();

    MessageHandler(const MessageHandler& other) = delete;
    MessageHandler& operator=(const MessageHandler& other) = delete;

    void cameraCallback(const sensor_msgs::ImageConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu& msg);
    void poseCallback(const geometry_msgs::PoseStamped& msg);

    static MessageHandler* instance_;
    ros::NodeHandle nh_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber imageSub_;

    ros::Subscriber imuSub_;
    ros::Subscriber poseSub_;

    PreProcessing preProcessing_;

    image_transport::Publisher preprocessedPub_;
    image_transport::Publisher bwPreprocessedPub_;
};

}

#endif // MESSAGE_HANDLER_H
