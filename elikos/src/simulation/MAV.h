#ifndef __MAV_H_
#define __MAV_H_

#include <elikos_lib/pid.hpp>
#include <visualization_msgs/Marker.h>

#define PI 3.14159265

class MAV {
public:
    MAV(double simulationSpeed, ros::Duration cycleTime);
    ~MAV();
    void setVelXYPID(double kp, double ki, double kd);
    void setVelZPID(double kp, double ki, double kd);
    void setVelXYMax(double vel);
    void setVelZMax(double vel);
    std::string getName();
    tf::Transform getTransform();
    void move();
    void collide();
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    visualization_msgs::Marker getVizMarker();
    visualization_msgs::Marker getSetpointMarker();
private:
    std::string Name;
    ros::Duration cycleTime;
    double x, y, z, yaw, z_sp, yaw_sp, vel_xy_max, vel_z, vel_z_sp, simSpeed;
    tf::Vector3 vel_xy, vel_xy_sp, xy_sp, direction;
    tf::Transform t;
    tf::Vector3 v;
    tf::Quaternion q;
    void refreshTransform();
    Pid<double>* vel_x_pid, * vel_y_pid, * vel_z_pid;

};

#endif /* __MAV_H_ */
