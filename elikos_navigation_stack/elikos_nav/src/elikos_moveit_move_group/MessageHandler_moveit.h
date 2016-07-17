#ifndef MESSAGE_HANDLER_moveit
#define MESSAGE_HANDLER_moveit

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <geometry_msgs/PoseArray.h>

class MessageHandler_moveit
{
public:
    MessageHandler_moveit();
    ~MessageHandler_moveit();
    void dispatchMessageTarget(const geometry_msgs::PoseStamped::ConstPtr &input);

    geometry_msgs::PoseStamped getTarget();
    bool getHasTarget();

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    geometry_msgs::PoseStamped inputTarget_;
    bool hasTarget_;
};

#endif /// MESSAGE_HANDLER_TTF
