#include <memory>
#include "MessageHandler_moveit.h"

int main(int argc, char* argv[])
{
    // ROS Init
    ros::init( argc, argv, "moveit_move_group" );

    //Attributes
    ros::NodeHandle nh_;
    ros::Publisher display_publisher_;

    //Asynchronous spinner to compute the motion plan
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Messagehandler to receive the goal pose.
    MessageHandler_moveit messageHandler;

    //Initialization
    display_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  	moveit::planning_interface::MoveGroup group_ = moveit::planning_interface::MoveGroup("elikos_moveit_quadrotor_group");
    moveit_msgs::DisplayTrajectory display_trajectory;

    //move_group settings
    group_.setPlanningTime(1.0);//In seconds
    //group_.setGoalTolerance (0.1);//In meters
    //The workspace represents the boundaries of the planning volume.
    group_.setWorkspace(-10,-10,0,10,10,10);

    std::vector<double> quad_variable_values;

    group_.getCurrentState()->copyJointGroupPositions(group_.getCurrentState()->getRobotModel()->getJointModelGroup(group_.getName()), quad_variable_values);

    //This is position and orientation for the quadcopter
    quad_variable_values[0]=2.0;//position.x
    quad_variable_values[1]=0.7;//position y
    quad_variable_values[2]=1.0;//position z
    quad_variable_values[3]=std::sqrt(2)/2;//quaternion x
    quad_variable_values[4]=0.0;//quaternion y
    quad_variable_values[5]=0.0;//quaternion z
    quad_variable_values[6]=std::sqrt(2)/2;//quaternion w

    //Plan the goal pose
    group_.setJointValueTarget(quad_variable_values);
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool  success = group_.plan(my_plan);
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");

    //Publish trajectory for rviz
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher_.publish(display_trajectory);

    //Plan and execute the trajectory
    group_.asyncMove();

    ros::Rate r(30);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

}
