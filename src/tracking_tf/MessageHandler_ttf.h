#ifndef MESSAGE_HANDLER_TTF
#define MESSAGE_HANDLER_TTF

#include <ros/ros.h>
#include <elikos_ros/RobotRaw.h>
#include <elikos_ros/RobotRawArray.h>


class MessageHandler_TTF
{
public:
    MessageHandler_TTF();
    ~MessageHandler_TTF();
    void dispatchMessage(const elikos_ros::RobotRawArray::ConstPtr &input);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
};

#endif /// MESSAGE_HANDLER_TTF

