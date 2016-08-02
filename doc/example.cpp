#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros/RCIn.h>
#include <tf/tf.h>

float x, y, z, yaw;
float xi, yi, zi ,yawi;
float xsp, ysp, zsp;
unsigned int offb_switch = 0;
geometry_msgs::PoseStamped posesp;
geometry_msgs::Quaternion yawsp;

void callback(const geometry_msgs::PoseStampedConstPtr pose)
{
    //ROS_INFO("Elikos_node: %f %f %f %f", pose->pose.position.x, pose->pose.position.y, pose->pose.position.z, tf::getYaw(pose->pose.orientation));
    posesp = *pose;
}

void rc_callback(const mavros::RCInConstPtr rc)
{
    // ROS_INFO("Elikos_node: %d %d", rc->channels[5],rc->channels[6]);
    offb_switch = rc->channels[5];
}

int main(int argc, char **argv) {
// Initialize the ROS system
    ros::init(argc, argv, "elikos_node");


    // Establish this program as a ROS node
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("mavros/position/local", 1, callback);
    ros::Subscriber rc_sub = nh.subscribe("mavros/rc/in", 1, rc_callback);

    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint/local_position", 1);

    ros::Rate r(5);
    while (ros::ok()) {
        if (offb_switch < 1200) {
            xi = posesp.pose.position.x;
            yi = posesp.pose.position.y;
            zi = posesp.pose.position.z;
        } else {
            xsp = xi + 1.0;
            ysp = yi;
            zsp = 1.0;

            posesp.pose.position.x = xsp;
            posesp.pose.position.y = ysp;
            posesp.pose.position.z = zsp;
            pub.publish(posesp);
            ROS_INFO("Sending setpoint %f %f %f", xsp, ysp, zsp);
        }

        ros::spinOnce();
        r.sleep();
    }


}
