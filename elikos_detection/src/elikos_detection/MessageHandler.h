#ifndef MESSAGE_HANDLER
#define MESSAGE_HANDLER

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <elikos_ros/RobotRaw.h>
#include <elikos_ros/RobotRawArray.h>
#include "TargetDetection/TargetDetection.h"

class MessageHandler
{
public:
    MessageHandler(string calibrationFilename);
    ~MessageHandler();
    void dispatchMessage(const sensor_msgs::ImageConstPtr &input);
    void dispatchCommand(const std_msgs::String::ConstPtr& input);
	void saveCalibration(string filename);
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber is_;
    ros::Publisher pub_;
    ros::Subscriber subRC_;
    TargetDetection detection_;

    image_transport::Publisher pubImages_;//debug only
    //image_transport::Publisher pubRed_;//debug only
    //image_transport::Publisher pubGreen_;//debug only
};

#endif /// MESSAGE_HANDLER
