//
// Created by tonio on 12/06/15.
//

#include <ros/ros.h>
#include <elikos_lib/RCReceiver.h>
#include "SetpointManager.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "elikos_navigator");
    ros::NodeHandle nh;
    RCReceiver rcReceiver(nh);
    SetpointManager spManager(nh);

    tf::TransformListener tf_listener;

    tf::StampedTransform mav_world;
    tf::StampedTransform mav_world_start;
    tf::StampedTransform target_robot;
    tf::Transform setpoint;

    tf::Vector3 mav_start_position;
    tf::Quaternion mav_start_rotation;

    ros::Rate r(10);
    while (ros::ok()) {
        // Get the MAV and target robot transforms
        try {
            tf_listener.lookupTransform("local_origin", "fcu", ros::Time(0), mav_world);
            tf_listener.lookupTransform("local_origin", "target_robot", ros::Time(0), target_robot);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            r.sleep();
            continue;
        }

        if (rcReceiver[OFFBOARD_SWITCH] < 1200) {
            // Save the starting transform
            mav_world_start = mav_world;
        }
        else {
            // Set origin of setpoint, keep starting altitude
            tf::Vector3 spOrigin(target_robot.getOrigin().getX(),
                                 target_robot.getOrigin().getY(),
                                 mav_world_start.getOrigin().getZ());
            setpoint.setOrigin(spOrigin);

            // Keep the starting rotation (only yaw is taken from mavros)
            setpoint.setRotation(mav_world_start.getRotation());

            // Broadcast the transform
            spManager.sendLocalPositionSetpointTF(setpoint);
        }

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}