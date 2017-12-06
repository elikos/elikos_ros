#ifndef MESSAGE_HANDLER
#define MESSAGE_HANDLER

#include "TargetDetection/TargetDetection.h"
#include <cv_bridge/cv_bridge.h>
#include <elikos_main/RobotRaw.h>
#include <elikos_main/RobotRawArray.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <elikos_remote_calib_client/Calibratable.h>
#include <elikos_remote_calib_client/CalibDetection.h>

class MessageHandler : public remote_calib::Calibratable<elikos_remote_calib_client::CalibDetection>
{
public:
    MessageHandler(string calibrationFilename);
    ~MessageHandler();
    void dispatchMessage(const sensor_msgs::ImageConstPtr& input);
    void dispatchCommand(const std_msgs::String::ConstPtr& input);
	void saveCalibration(string filename);

    virtual void calibrate(const elikos_remote_calib_client::CalibDetection* const message);
    virtual void loadCalibration(const YAML::Node& fileContent);
    virtual void saveCalibration(YAML::Node& fileContent);

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber is_;

    ros::Publisher pub_;
    ros::Publisher pubCommandOutput_;
    ros::Subscriber subRC_;
    TargetDetection detection_;

    image_transport::Publisher pubImages_;//debug only
    image_transport::Publisher pubRed_;//debug only
    image_transport::Publisher pubGreen_;//debug only
};

#endif /// MESSAGE_HANDLER
