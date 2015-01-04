/*
 * GroundRobot.cpp
 *
 *  Created on: Jan 3, 2015
 *      Author: Antonio Sanniravong
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <elikos_ros/GroundRobot.h>

GroundRobot::GroundRobot(int id){
	robotID = id;
	setStartingPose();
	setName();
	lastAutoReverse = ros::Time::now();
	lastNoise = ros::Time::now();
	srand(time(NULL));
};

GroundRobot::~GroundRobot(){};

void GroundRobot::setStartingPose(){
	x = (5/PI)*cos(robotID*PI/5);
	y = (5/PI)*sin(robotID*PI/5);
	z = 0;
	yaw = robotID*PI/5;
	refreshTransform();
}

void GroundRobot::setName(){
	std::stringstream ss;
	ss << "robot" << robotID;
	robotName = ss.str();
}

std::string GroundRobot::getName(){
	return robotName;
}

tf::Transform GroundRobot::getTransform(){
	return t;
}

void GroundRobot::advance(ros::Duration cycleTime){

	if ((ros::Time::now()-lastAutoReverse).toSec() > 20.0 && !isSpinning) {
		autoReverse();
	}

	if ((ros::Time::now()-lastNoise).toSec() > 5.0 && !isSpinning) {
		noise();
	}

	if (!isSpinning){
		x += 0.33 * cycleTime.toSec() * cos(yaw);
		y += 0.33 * cycleTime.toSec() * sin(yaw);
	}

	if (turnAngle) {
		yaw += limitTurn(turnAngle, (PI/2.456), cycleTime.toSec());
	} else {
		isSpinning = false;
	}

	refreshTransform();
}

void GroundRobot::reverse(){
	turnAngle -= PI; // 180 Degrees clockwise
	isSpinning = true;
}

void GroundRobot::autoReverse(){
	lastAutoReverse = ros::Time::now();
	reverse();
}

void GroundRobot::noise(){
	lastNoise = ros::Time::now();
	turnAngle += (PI/9) * (rand()%1000 - 500)/500; // Max 20 degree deviation
}

void GroundRobot::touch(){
	turnAngle = PI/4; // 45 Degrees clockwise
	isSpinning = true;
}

void GroundRobot::collide(){
	if (!isSpinning){
		reverse();
	}
}

void GroundRobot::refreshTransform(){
	v.setX(x);
	v.setY(y);
	v.setZ(z);
	q.setRPY(0, 0, yaw);
	t.setOrigin(v);
	t.setRotation(q);
}

double GroundRobot::limitTurn(double& angle, double angularSpeed, double cycleDuration){
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
