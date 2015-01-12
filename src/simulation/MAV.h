#ifndef __MAV_H_
#define __MAV_H_

#include <elikos_lib/pid.hpp>

#define PI 3.14159265

class MAV {
public:
    MAV(double simulationSpeed);
    ~MAV();
    void setVelXYPID(double kp, double ki, double kd, ros::Duration cycleTime);
    void setVelZPID(double kp, double ki, double kd, ros::Duration cycleTime);
    void setVelXYMax(double vel);
    void setVelZMax(double vel);
    std::string getName();
    tf::Transform getTransform();
    void move(ros::Duration cycleTime);
    void collide();
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
private:
    std::string Name;
    double x, y, z, yaw, z_sp, yaw_sp, vel_xy_max, vel_z, vel_z_sp, simSpeed;
    tf::Vector3 vel_xy, vel_xy_sp, xy_sp, direction;
    tf::Transform t;
    tf::Vector3 v;
    tf::Quaternion q;
    void refreshTransform();
    Pid<double>* vel_x_pid, * vel_y_pid, * vel_z_pid;

};

#endif /* __MAV_H_ */
