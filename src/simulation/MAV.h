/*
 * MAV.h
 *
 *  Created on: Jan 9, 2015
 *      Author: Antonio Sanniravong
 */

#ifndef __MAV_H_
#define __MAV_H_

#include <elikos_lib/pid.hpp>

#define PI 3.14159265

class MAV {
public:
    MAV(int id, double simulationSpeed);
    ~MAV();
    void move(ros::Duration cycleTime);
    void collide();
    std::string getName();
    tf::Transform getTransform();
    int getID();
    void setVelXYPID(double kp, double ki, double kd, ros::Duration cycleTime);
    void setVelZPID(double kp, double ki, double kd, ros::Duration cycleTime);
    void setVelXYMax(double vel);
    void setVelZMax(double vel);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
private:
    int ID;
    std::string Name;
    bool isLanded;
    double x, y, z, yaw, vel_xy_max, simSpeed;
    double z_sp, yaw_sp;
    tf::Vector3 vel_xy, vel_xy_sp, xy_sp, direction;
    double vel_z, vel_z_sp;
    tf::Transform t;
    tf::Vector3 v;
    tf::Quaternion q;
    void refreshTransform();
    Pid<double>* vel_x_pid, * vel_y_pid, * vel_z_pid;

};

#endif /* __MAV_H_ */
