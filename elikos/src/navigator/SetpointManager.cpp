//
// Created by tonio on 09/06/15.
//

#include "SetpointManager.h"

SetpointManager::SetpointManager(ros::NodeHandle nh) : _nh(&nh) {
    _setpoint_pub = _nh->advertise<geometry_msgs::PoseStamped>(TOPIC_NAMES[mavros_setpoint_local_position], 1);
}

SetpointManager::~SetpointManager() {}

void SetpointManager::sendSetpoint(const tf::Transform &t) {
    double yaw = tf::getYaw(t.getRotation());
    if (isnan(yaw)) {
        throw "Yaw is NaN.";
    }
    geometry_msgs::Pose pose;
    tf::poseTFToMsg(t, pose);
    publishPoseStamped(pose);
}

void SetpointManager::sendSetpoint(const tf::Vector3 &v, const tf::Quaternion &q) {
    tf::Transform t;

    // Set Position
    t.setOrigin(v);

    // Set Rotation
    t.setRotation(q);

    // Send
    sendSetpoint(t);
}

void SetpointManager::sendSetpoint(const tf::Vector3 &v, const double yaw) {
    tf::Transform t;

    // Set Position
    t.setOrigin(v);

    // Set Rotation (from yaw)
    tf::Quaternion q;
    q.setRPY(0, 0, yaw);
    t.setRotation(q);

    // Send
    sendSetpoint(t);
}

void SetpointManager::sendSetpoint(const double x, const double y, const double z, tf::Quaternion &q) {
    tf::Transform t;

    // Set Position
    t.setOrigin(tf::Vector3(x, y, z));

    // Set Rotation
    t.setRotation(q);

    // Send
    sendSetpoint(t);
}

void SetpointManager::sendSetpoint(const double x, const double y, const double z, const double yaw) {
    tf::Transform t;

    // Set Position
    t.setOrigin(tf::Vector3(x, y, z));

    // Set Rotation (from yaw)
    tf::Quaternion q;
    q.setRPY(0, 0, yaw);
    t.setRotation(q);

    // Send
    sendSetpoint(t);
}

void SetpointManager::publishPoseStamped(const geometry_msgs::Pose &pose) {
    geometry_msgs::PoseStamped setpoint;
    setpoint.pose = pose;
    setpoint.header.stamp = ros::Time::now();
    _setpoint_pub.publish(setpoint);
}
