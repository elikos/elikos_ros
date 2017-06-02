#include <memory>
#include "MessageHandler_moveit.h"

int main(int argc, char* argv[])
{
    // ROS Init
    ros::init( argc, argv, "moveit_move_group" );

    ros::NodeHandle nh;
    ros::Publisher pub;
    pub = nh.advertise<elikos_ros::TrajectoryCmd>("elikos_trajectory", 1);

    //Asynchronous spinner to compute the motion plan
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Messagehandler to receive the goal pose.
    MessageHandler_moveit messageHandler;

    Moveit_move_group move_group_;

    ros::Rate r(10);
    while(ros::ok())
    {
        if(messageHandler.hasNewMessage())
        {
            elikos_ros::AICmd ai_cmd = messageHandler.getAICmd();
            elikos_ros::TrajectoryCmd traj_cmd;
            traj_cmd.cmdCode = ai_cmd.cmdCode;

            if (ai_cmd.cmdCode == CmdCode::MOVE_TO_POINT)
            {
                traj_cmd.trajectory = move_group_.move(ai_cmd.pose);
            }

            pub.publish(traj_cmd);
        }
        ros::spinOnce();
        r.sleep();
    }

}
