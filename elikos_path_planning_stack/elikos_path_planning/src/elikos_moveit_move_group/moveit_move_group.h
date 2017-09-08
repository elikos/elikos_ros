#ifndef MOVEIT_MOVE_GROUP
#define MOVEIT_MOVE_GROUP

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <geometry_msgs/PoseArray.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <std_srvs/Empty.h>
#include <ctime>

#include <elikos_ros/TrajectoryCmd.h>
#include <elikos_ros/AICmd.h>

#include <CmdDefines.h>

class Moveit_move_group
{
public:
    Moveit_move_group();
    ~Moveit_move_group();

    trajectory_msgs::MultiDOFJointTrajectory move(geometry_msgs::PoseStamped target);
    void publishTrajectoryPoint(geometry_msgs::Transform_<std::allocator<void> > trajectoryPoint);

private:
    moveit::planning_interface::MoveGroup group_ = moveit::planning_interface::MoveGroup("elikos_moveit_quadrotor_group");

    tf::TransformListener tf_listener_;

    std::string parent_frame_;
  	std::string child_frame_;
  	double toleranceFreeOctomap_;
  	double toleranceAchieveGoal_;

    double safetyTime_;

    ros::NodeHandle nh_;
};

#endif
