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
    vel_xy_pid = new Pid<double>(0.0, 0.0, 0.0, Pid<double>::PID_DIRECT, Pid<double>::DONT_ACCUMULATE_OUTPUT, 33.0, 0.0, 0.0, 0.0);
    vel_z_pid = new Pid<double>(0.0, 0.0, 0.0, Pid<double>::PID_DIRECT, Pid<double>::DONT_ACCUMULATE_OUTPUT, 33.0, 0.0, 0.0, 0.0);
    this->refreshTransform();
};

MAV::~MAV(){
    delete vel_xy_pid;
    delete vel_z_pid;
}

void MAV::setVelXYPID(double kp, double ki, double kd, ros::Duration cycleTime){
    vel_xy_pid->SetTunings(kp, ki, kd);
    vel_xy_pid->SetSamplePeriod(cycleTime.toSec() * 1000);
}

void MAV::setVelZPID(double kp, double ki, double kd, ros::Duration cycleTime){
    vel_z_pid->SetTunings(kp, ki, kd);
    vel_z_pid->SetSamplePeriod(cycleTime.toSec() * 1000);
}

void MAV::setPosTarget(){
    // TODO
}

void MAV::setVelTarget(){
    // TODO
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

    this->refreshTransform();
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

double MAV::limitTurn(double& angle, double angularSpeed, double cycleDuration){
    double limitedAngle;
    double maxAngle = angularSpeed * cycleDuration;
    if (fabs(angle) > maxAngle){
        limitedAngle = (angle > 0) ? maxAngle : -maxAngle;
        angle -= limitedAngle;
    } else {
        limitedAngle = angle;
        angle = 0;
    }
    return limitedAngle;
}
