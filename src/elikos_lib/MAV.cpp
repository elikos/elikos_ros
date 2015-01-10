/*
 * MAV.cpp
 *
 *  Created on: Jan 3, 2015
 *      Author: Antonio Sanniravong
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <elikos_lib/MAV.h>
#include <elikos_lib/GroundRobot.h>
#include <elikos_lib/pid.hpp>

MAV::MAV(int id, double simulationSpeed) : Vel_X_PID(0.0, 0.0, 0.0, 0, 33.3, 0.0, 5.0, 0){
	ID = id;
	simSpeed = simulationSpeed;
	setStartingPose();
	setName();
};

MAV::~MAV(){};

void MAV::setStartingPose(){
	x = 0;
	y = 0;
	z = 0;
	yaw = 0;

	refreshTransform();
}

void MAV::setName(){
	std::stringstream ss;
	ss << "MAV" << ID;
	Name = ss.str();
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

void MAV::advance(ros::Duration cycleTime){

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
