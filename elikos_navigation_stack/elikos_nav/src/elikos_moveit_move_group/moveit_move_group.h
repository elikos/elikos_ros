#ifndef MOVEIT_MOVE_GROUP
#define MOVEIT_MOVE_GROUP

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <geometry_msgs/PoseArray.h>

class Moveit_move_group
{
public:
    Moveit_move_group();
    ~Moveit_move_group();

    void move(geometry_msgs::PoseStamped target);

private:
    moveit::planning_interface::MoveGroup group_ = moveit::planning_interface::MoveGroup("elikos_moveit_quadrotor_group");
};

#endif
