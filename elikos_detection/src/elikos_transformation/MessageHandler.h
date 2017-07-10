#ifndef MESSAGE_HANDLER
#define MESSAGE_HANDLER

#include <ros/ros.h>
#include <elikos_ros/RobotRaw.h>
#include <elikos_ros/RobotRawArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <elikos_ros/TargetRobotArray.h>
#include <elikos_ros/TargetRobot.h>
#include <string>
#include <sstream>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <opencv2/opencv.hpp>

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
