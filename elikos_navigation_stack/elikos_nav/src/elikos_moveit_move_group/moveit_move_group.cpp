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
  	moveit::planning_interface::MoveGroup group_ = moveit::planning_interface::MoveGroup("elikos_moveit_quadrotor_group");

    //move_group settings
    group_.setPlanningTime(1.0);//In seconds
    //group_.setGoalTolerance (0.1);//In meters
    //The workspace represents the boundaries of the planning volume.
    group_.setWorkspace(-10,-10,0,10,10,3);

    group_.allowReplanning(true);

    ros::Rate r(10);
    int i = 0;
    while(ros::ok())
    {
        if(messageHandler.getHasTarget() || true)
        {
          geometry_msgs::PoseStamped target = messageHandler.getTarget();

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
          /*quad_variable_values[0] = 0;//position.x
          quad_variable_values[1] = 10;//position y
          quad_variable_values[2] = -5.0;//position z
          quad_variable_values[3] = 0;//quaternion x
          quad_variable_values[4] = 0;//quaternion y
          quad_variable_values[5] = 0;//quaternion z
          quad_variable_values[6] = 1;//quaternion w*/

          //Plan the goal pose
          group_.setJointValueTarget(quad_variable_values);

          //Plan and execute the trajectory
          group_.asyncMove();

          i++;
        }
        ros::spinOnce();
        r.sleep();
    }


}
