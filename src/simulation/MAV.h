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
    void setPosTarget(double x, double y, double z);
    void setVelTarget();
private:
    int ID;
    std::string Name;
    bool isLanded;
    double x, y, z, yaw, simSpeed;
    double x_sp, y_sp, z_sp, yaw_sp;
    double vel_x, vel_y, vel_z;
    double vel_x_sp, vel_y_sp, vel_z_sp;
    tf::Transform t;
    tf::Vector3 v;
    tf::Quaternion q;
    void refreshTransform();
    double limitTurn(double& angle, double angularSpeed, double cycleDuration);
    Pid<double>* vel_xy_pid, * vel_z_pid;

};

#endif /* __MAV_H_ */
