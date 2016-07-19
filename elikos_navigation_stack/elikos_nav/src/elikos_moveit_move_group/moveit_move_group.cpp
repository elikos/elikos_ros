
#include "moveit_move_group.h"

Moveit_move_group::Moveit_move_group()
{
  //move_group settings
  group_.setPlanningTime(1.0);//In seconds
  //group_.setGoalTolerance (0.1);//In meters
  //The workspace represents the boundaries of the planning volume.
  group_.setWorkspace(-10,-10,0,10,10,3);

  group_.allowReplanning(true);
}

Moveit_move_group::~Moveit_move_group()
{
}

void Moveit_move_group::move(geometry_msgs::PoseStamped target)
{
  group_.stop();

  std::vector<double> quad_variable_values;

  group_.getCurrentState()->copyJointGroupPositions(group_.getCurrentState()->getRobotModel()->getJointModelGroup(group_.getName()), quad_variable_values);

  //This is the new position and orientation for the quadcopter
  quad_variable_values[0] = target.pose.position.x;//position.x
  quad_variable_values[1] = target.pose.position.y;//position y
  quad_variable_values[2] = target.pose.position.z;//position z
  quad_variable_values[3] = target.pose.orientation.x;//quaternion x
  quad_variable_values[4] = target.pose.orientation.y;//quaternion y
  quad_variable_values[5] = target.pose.orientation.z;//quaternion z
  quad_variable_values[6] = target.pose.orientation.w;//quaternion w

  //Plan the goal pose
  group_.setJointValueTarget(quad_variable_values);

  //Plan and execute the trajectory
  group_.asyncMove();
}
