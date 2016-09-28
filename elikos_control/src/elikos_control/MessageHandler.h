#ifndef MESSAGE_HANDLER
#define MESSAGE_HANDLER

#include <ros/ros.h>
#include <elikos_ros/TrajectoryCmd.h>

class MessageHandler
{
public:
    MessageHandler();
    ~MessageHandler();
    void dispatchMessage(const elikos_ros::TrajectoryCmd::ConstPtr &input);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
};

#endif /// MESSAGE_HANDLER
