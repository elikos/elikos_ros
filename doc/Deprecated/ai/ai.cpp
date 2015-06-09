/**
* @brief    Artificial intelligence ROS node main program for Élikos project.
*/

#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "./../defines.cpp"


geometry_msgs::PoseStamped getPoseStamped(float angle);


int main(int argc, char **argv) {
    // Initialize the ROS system
    ros::init(argc, argv, "elikos_ai");

    // Establish this program as a ROS node
    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>(TOPICS_NAMES[mavros_setpoint_local_position], 1);
    ros::Rate r(30);    //10 hz

    float angle = 0.0;

    while (ros::ok()) {
        geometry_msgs::PoseStamped pose = getPoseStamped(angle);
        angle += PI/120;
        if(angle >= 2*PI)
            angle = 0.0;

        while (pose_pub.getNumSubscribers() < 1){
            if(!ros::ok()) return 0;
            ROS_WARN_ONCE("Please create subscriber to the pose");
            sleep(1);
        }

        pose_pub.publish(pose);

        r.sleep();
    }
}


geometry_msgs::PoseStamped getPoseStamped(float angle){
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = cos(angle);
    pose.pose.position.y = sin(2*angle)/2;
    pose.pose.position.z = 1.5 + sin(3*angle)/2;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 0.0;

    return pose;
}

