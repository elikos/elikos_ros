
#include "moveit_move_group.h"

Moveit_move_group::Moveit_move_group():
  parent_frame_("elikos_arena_origin"),
  child_frame_("elikos_setpoint"),
  toleranceAchieveGoal_(0.5),
  toleranceFreeOctomap_(0.1),
  safetyTime_(3.0)
{
  //move_group settings
  group_.setPlanningTime(1.0);//In seconds

  //The workspace represents the boundaries of the planning volume.
  group_.setWorkspace(-10,-10,0,10,10,3);

  group_.allowReplanning(true);

  group_.setNumPlanningAttempts(20);

}

Moveit_move_group::~Moveit_move_group()
{
}

trajectory_msgs::MultiDOFJointTrajectory Moveit_move_group::move(geometry_msgs::PoseStamped target)
{

  std::vector<double> quad_variable_values;

  group_.getCurrentState()->copyJointGroupPositions(group_.getCurrentState()->getRobotModel()->getJointModelGroup(group_.getName()), quad_variable_values);

  //This is the new position and orientation for the quadcopter
  quad_variable_values[0] = target.pose.position.x;//position.x
  quad_variable_values[1] = target.pose.position.y;//position y
  quad_variable_values[2] = target.pose.position.z;//position z
  quad_variable_values[3] = 0;//target.pose.orientation.x;//quaternion x
  quad_variable_values[4] = 0;//target.pose.orientation.y;//quaternion y
  quad_variable_values[5] = 0;//target.pose.orientation.z;//quaternion z
  quad_variable_values[6] = 1;//target.pose.orientation.w;//quaternion w

  //Plan the goal pose
  group_.setJointValueTarget(quad_variable_values);

  //Plan creation
  moveit::planning_interface::MoveGroup::Plan plan;

  try
  {
    //Current position
    tf::StampedTransform currentPosition;
    tf_listener_.lookupTransform(parent_frame_, "elikos_base_link",
                              ros::Time(0), currentPosition);
    //If the drone is far enough of his position 
    if( pow(target.pose.position.x-currentPosition.getOrigin().x(), 2)+
        pow(target.pose.position.y-currentPosition.getOrigin().y(), 2)+
        pow(target.pose.position.z-currentPosition.getOrigin().z(), 2) > pow(toleranceAchieveGoal_, 2))
    {

      group_.setStartStateToCurrentState();

      //Planning
      moveit_msgs::MoveItErrorCodes err = group_.plan(plan);

      if(err.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
      {
        //Computation of time stamp and population of the velocities in the trajectory.
        moveit_msgs::RobotTrajectory trajectory_msg = plan.trajectory_;
        
        robot_trajectory::RobotTrajectory rt(group_.getCurrentState()->getRobotModel(), "elikos_moveit_quadrotor_group");

        rt.setRobotTrajectoryMsg(*group_.getCurrentState(), trajectory_msg);
        
        trajectory_processing::IterativeParabolicTimeParameterization iptp;

        bool success = iptp.computeTimeStamps(rt);
        ROS_ERROR("Computed time stamp on trajectory %s",success?"SUCCEDED":"FAILED");

        rt.getRobotTrajectoryMsg(trajectory_msg);
        return trajectory_msg.multi_dof_joint_trajectory;

        // Publish TrajectoryCmd message on "elikos_trajectory".
        // elikos_ros::TrajectoryCmd cmd;
        // cmd.cmdCode = 0;
        // cmd.trajectory = trajectory;
        // pub_.publish(cmd);
      }
      else
      {
        // No planning available.
        ROS_ERROR_STREAM("Safety mode!");
        // TODO : Feedback
        std_srvs::Empty::Request req;
        std_srvs::Empty::Response res;
        ros::service::call("/clear_octomap", req, res);
      }
    }

  }
  catch (tf::TransformException ex){
   ROS_ERROR("%s",ex.what());
   ros::Duration(1.0).sleep();
  }
}

