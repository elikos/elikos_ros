#ifndef MESSAGE_HANDLER
#define MESSAGE_HANDLER

#include <ros/ros.h>
#include <elikos_ros/RobotRaw.h>
#include <elikos_ros/RobotRawArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>

#include "TransformationUnit.h"

class MessageHandler
{
public:
    MessageHandler();
    ~MessageHandler();
    void dispatchMessageRobotRaw(const elikos_ros::RobotRawArray::ConstPtr &input);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Publisher pubTest_;
    elikos_ros::RobotRawArray newArray_;
};

#endif /// MESSAGE_HANDLER_TTF
