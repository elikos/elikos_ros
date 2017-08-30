#ifndef MESSAGE_HANDLER
#define MESSAGE_HANDLER

#include <ros/ros.h>
#include <elikos_main/RobotRaw.h>
#include <elikos_main/RobotRawArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>

#include "TransformationUnit.h"

class MessageHandler
{
public:
    MessageHandler();
    ~MessageHandler();
    void dispatchMessageRobotRaw(const elikos_main::RobotRawArray::ConstPtr &input);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pubTest_;
    elikos_main::RobotRawArray newArray_;
	TransformationUnit transformationUnit_;
};

#endif /// MESSAGE_HANDLER_TTF
