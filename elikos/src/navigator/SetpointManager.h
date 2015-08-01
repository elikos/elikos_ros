//
// Created by tonio on 09/06/15.
//

#ifndef PROJECT_SETPOINTMANAGER_H
#define PROJECT_SETPOINTMANAGER_H

#include <ros/ros.h>
#include "./../defines.cpp"
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

class SetpointManager {
public:
    SetpointManager(ros::NodeHandle &nh);
    ~SetpointManager();

    //not const because yolo
    void sendLocalPositionSetpoint(tf::Transform &t);
    void sendLocalPositionSetpoint(const tf::Vector3 &p, const tf::Quaternion &q);
    void sendLocalPositionSetpoint(const tf::Vector3 &p, const double yaw);
    void sendLocalPositionSetpoint(const double x, const double y, const double z, tf::Quaternion &q);
    void sendLocalPositionSetpoint(const double x, const double y, const double z, const double yaw);

    void sendLocalPositionSetpointTF(const tf::Transform &t);

private:
    void publishPoseStamped(geometry_msgs::Pose const &pose);
    ros::NodeHandle* nh_;
    ros::Publisher setpoint_pub_;
    tf::TransformBroadcaster tf_broadcaster_;
};


#endif //PROJECT_SETPOINTMANAGER_H

