#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>


bool local_pos_valid = false;
bool vo_valid = false;
geometry_msgs::PoseStamped local_pose;
geometry_msgs::Pose local_pose_offset;
geometry_msgs::PoseStamped pose_with_offset;
geometry_msgs::PoseWithCovarianceStamped vo_pose;
ros::Time last_vo;
ros::Time last_local_pos;

const ros::Duration timeout(0.5);

void local_pos_callback(const geometry_msgs::PoseStampedConstPtr pose){
    if (!local_pos_valid) {
        ROS_WARN("Local Pose valid!");
    }
    local_pose = *pose;
    local_pos_valid = true;
    last_local_pos = ros::Time::now();
}

void vo_pose_callback(const geometry_msgs::PoseWithCovarianceStampedPtr pose){
    if (!vo_valid) {
        ROS_WARN("VO valid! Current offset: %f, %f, %f.",
                 local_pose_offset.position.x,local_pose_offset.position.y, local_pose_offset.position.z);
    }
    vo_pose = *pose;
    vo_valid = true;
    last_vo = ros::Time::now();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_offset");
    ros::NodeHandle nh;
    ros::Rate rate(100);

    ros::Subscriber local_pos_sub = nh.subscribe("mavros/local_position/local", 1, local_pos_callback);
    ros::Subscriber vo_sub = nh.subscribe("svo/pose", 1, vo_pose_callback);

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 1);

    last_vo = ros::Time(0);
    last_local_pos = ros::Time(0);

    local_pose_offset.position.x = 0;
    local_pose_offset.position.y = 0;
    local_pose_offset.position.z = 0;
    local_pose_offset.orientation.x = 0;
    local_pose_offset.orientation.y = 0;
    local_pose_offset.orientation.z = 0;
    local_pose_offset.orientation.w = 1;

    while(ros::ok()){
        if(ros::Time::now() - last_local_pos > timeout){
            if (local_pos_valid) {
                ROS_ERROR("Local Pose timed out!");
            }
            local_pos_valid = false;
        }

        if(ros::Time::now() - last_vo > timeout && local_pos_valid){
            if (vo_valid) {
                ROS_ERROR("VO timed out! Resetting offset.");
            }
            vo_valid = false;
            local_pose_offset.position = local_pose.pose.position;
            local_pose_offset.orientation = local_pose.pose.orientation;
        }

        if(local_pos_valid && vo_valid){
            pose_with_offset.pose.position.x = local_pose_offset.position.x + vo_pose.pose.pose.position.x;
            pose_with_offset.pose.position.y = local_pose_offset.position.y + vo_pose.pose.pose.position.y;
            pose_with_offset.pose.position.z = local_pose_offset.position.z + vo_pose.pose.pose.position.z;
            pose_with_offset.pose.orientation = vo_pose.pose.pose.orientation;
            pose_with_offset.header.stamp = ros::Time::now();
            pose_with_offset.header.frame_id = "local_origin";

            pose_pub.publish(pose_with_offset);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}