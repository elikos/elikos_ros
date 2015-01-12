/*
 * MAV.cpp
 *
 *  Created on: Jan 3, 2015
 *      Author: Antonio Sanniravong
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include "MAV.h"
#include <elikos_lib/pid.hpp>

MAV::MAV(int id, double simulationSpeed){
    x = 0;
    y = 0;
    z = 0;
    yaw = 0;
    isLanded = false;
    ID = id;
    simSpeed = simulationSpeed;
    Name = "MAV";
    vel_x_pid = new Pid<double>(0.0, 0.0, 0.0, // PID
                                Pid<double>::PID_DIRECT, // Controller direction
                                Pid<double>::DONT_ACCUMULATE_OUTPUT, // Output mode
                                33.3 / simSpeed, // Sample period
                                0.0, 5.0, 0.0); // Min output, Max output, Setpoint
    vel_y_pid = new Pid<double>(0.0, 0.0, 0.0,
                                Pid<double>::PID_DIRECT,
                                Pid<double>::DONT_ACCUMULATE_OUTPUT,
                                33.3 / simSpeed,
                                0.0, 5.0, 0.0);
    vel_z_pid = new Pid<double>(0.0, 0.0, 0.0,
                                Pid<double>::PID_DIRECT,
                                Pid<double>::DONT_ACCUMULATE_OUTPUT,
                                33.3 / simSpeed,
                                0.0, 5.0, 0.0);
    refreshTransform();
};

MAV::~MAV(){
    delete vel_x_pid;
    delete vel_y_pid;
    delete vel_z_pid;
    vel_x_pid = NULL;
    vel_y_pid = NULL;
    vel_z_pid = NULL;
}

void MAV::setVelXYPID(double kp, double ki, double kd, ros::Duration cycleTime){
    vel_x_pid->SetTunings(kp, ki, kd);
    vel_x_pid->SetSamplePeriod((cycleTime.toSec() / simSpeed) * 1000);
    vel_y_pid->SetTunings(kp, ki, kd);
    vel_y_pid->SetSamplePeriod((cycleTime.toSec() / simSpeed) * 1000);
}

void MAV::setVelZPID(double kp, double ki, double kd, ros::Duration cycleTime){
    vel_z_pid->SetTunings(kp, ki, kd);
    vel_z_pid->SetSamplePeriod((cycleTime.toSec() / simSpeed) * 1000);
}

void MAV::setVelXYMax(double vel){
    vel_xy_max = vel;
    vel_x_pid->SetOutputLimits(-vel, vel);
    vel_y_pid->SetOutputLimits(-vel, vel);
}

void MAV::setVelZMax(double vel){
    vel_z_pid->SetOutputLimits(0.0, vel);
}

void MAV::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    x_sp = msg->pose.position.x;
    y_sp = msg->pose.position.y;
    z_sp = msg->pose.position.z;
    // TODO: Yaw

}

std::string MAV::getName(){
    return Name;
}

tf::Transform MAV::getTransform(){
    return t;
}

int MAV::getID(){
    return ID;
}

void MAV::move(ros::Duration cycleTime){
    // Generate velocity setpoint
        // XY
    vel_xy_sp.setX(x_sp - x);
    vel_xy_sp.setY(y_sp - y);
    vel_xy_sp.normalize();
        // Z
    vel_z_sp = z_sp - z;
    if (fabs(vel_z_sp) > 1.0) {
        vel_z_sp /= fabs(vel_z_sp);
    }

    // Compute new velocities
        // XY
    vel_x_pid->Run(vel_xy_sp.getX());
    vel_y_pid->Run(vel_xy_sp.getY());
    vel_xy.setX(vel_x_pid->output);
    vel_xy.setY(vel_y_pid->output);
    if (vel_xy.length() > vel_xy_max) {
        vel_xy.normalize();
        vel_xy *= vel_xy_max;
    }
        // Z
    vel_z_pid->Run(vel_z_sp);
    vel_z = vel_z_pid->output;

    // Set new position
    x += vel_xy.getX() * cycleTime.toSec() * simSpeed;
    y += vel_xy.getY() * cycleTime.toSec() * simSpeed;
    z += vel_z * cycleTime.toSec() * simSpeed;
    // TODO: Yaw
    refreshTransform();
}

void MAV::collide(){
    // TODO
}

void MAV::refreshTransform(){
    v.setX(x);
    v.setY(y);
    v.setZ(z);
    q.setRPY(0, 0, yaw);
    t.setOrigin(v);
    t.setRotation(q);
}
