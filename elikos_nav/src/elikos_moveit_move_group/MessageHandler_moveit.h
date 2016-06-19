#ifndef MESSAGE_HANDLER_moveit
#define MESSAGE_HANDLER_moveit

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

class MessageHandler_moveit
{
public:
    MessageHandler_moveit();
    ~MessageHandler_moveit();

private:
    ros::NodeHandle nh_;
};

#endif /// MESSAGE_HANDLER_TTF
