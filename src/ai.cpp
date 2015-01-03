/**
* @brief    Artificial intelligence ROS node main program for Élikos project.
*/

#include <cmath>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#define PI 3.14159265


visualization_msgs::Marker getMarker(float angle);

int main(int argc, char **argv) {
    // Initialize the ROS system
    ros::init(argc, argv, "elikos_ai_hello");

    // Establish this program as a ROS node
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Rate r(30);    //10 hz
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "/frame_elikos";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 0.0;

    float angle = 0.0;

    while (ros::ok()) {
        visualization_msgs::Marker marker = getMarker(angle);
        angle += PI/120;
        if(angle >= 2*PI)
            angle = 0.0;

        while (marker_pub.getNumSubscribers() < 1){
            if(!ros::ok()) return 0;
            ROS_WARN_ONCE("Please create subscriber to the marker");
            sleep(1);
        }

        marker_pub.publish(marker);

        r.sleep();
    }
}


visualization_msgs::Marker getMarker(float angle){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/my_header";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    //Lemniscate of Gerono
    marker.pose.position.x = cos(angle);
    marker.pose.position.y = sin(2*angle)/2;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.1;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    return marker;
}
