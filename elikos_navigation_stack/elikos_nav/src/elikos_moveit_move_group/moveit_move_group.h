#ifndef MOVEIT_MOVE_GROUP
#define MOVEIT_MOVE_GROUP

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <moveit_msgs/MoveItErrorCodes.h>

#include <geometry_msgs/PoseArray.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <opencv2/opencv.hpp>

#include <std_srvs/Empty.h>

#ifndef PI
#define PI 3.14159265
#endif
class Moveit_move_group
{
public:
    Moveit_move_group();
    ~Moveit_move_group();

    void move(geometry_msgs::PoseStamped target);
    void publishTrajectoryPoint(geometry_msgs::Transform_<std::allocator<void> > trajectoryPoint);

private:
    moveit::planning_interface::MoveGroup group_ = moveit::planning_interface::MoveGroup("elikos_moveit_quadrotor_group");

    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener listener;

    std::string parent_frame_;
  	std::string child_frame_;
  	double toleranceNextGoal_;
  	double toleranceFreeOctomap_;
  	double toleranceAchieveGoal_;
    geometry_msgs::Transform_<std::allocator<void> > trajectoryPoint_;

    bool isFirst_;
};

#endif
