//
// Created by tonio on 09/06/15.
//

#ifndef PROJECT_SETPOINTMANAGER_H
#define PROJECT_SETPOINTMANAGER_H

#include <ros/ros.h>
#include "./../defines.cpp"
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

class SetpointManager {
public:
    SetpointManager(ros::NodeHandle nh);
    ~SetpointManager();

    void sendSetpoint(const tf::Transform &t);
    void sendSetpoint(const tf::Vector3 &p, const tf::Quaternion &q);
    void sendSetpoint(const tf::Vector3 &p, const double yaw);
    void sendSetpoint(const double x, const double y, const double z, tf::Quaternion &q);
    void sendSetpoint(const double x, const double y, const double z, const double yaw);

private:
    void publishPoseStamped(geometry_msgs::Pose const &pose);
    ros::NodeHandle* _nh;
    ros::Publisher _setpoint_pub;
};


#endif //PROJECT_SETPOINTMANAGER_H

