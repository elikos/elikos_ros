#include <memory>
#include "MessageHandler_moveit.h"

int main(int argc, char* argv[])
{
    // ROS Init
    ros::init( argc, argv, "moveit_move_group" );

    //Attributes
    ros::NodeHandle nh_;

    //Asynchronous spinner to compute the motion plan
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Messagehandler to receive the goal pose.
    MessageHandler_moveit messageHandler;

    Moveit_move_group move_group_;

    ros::Rate r(10);
    while(ros::ok())
    {;
        if(messageHandler.hasTarget())
        {
          geometry_msgs::PoseStamped goal = messageHandler.getTarget();
          move_group_.move(goal);
        }
        ros::spinOnce();
        r.sleep();
    }

}
