/*
 * GroundRobot.cpp
 *
 *  Created on: Jan 3, 2015
 *      Author: Antonio Sanniravong
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <elikos_ros/GroundRobot.h>

GroundRobot::GroundRobot(robot_type type, int nbRobots, int id, double simulationSpeed){
	robotID = id;
	robotType = type;
	simSpeed = simulationSpeed;
	nRobots = nbRobots;
	setStartingPose();
	setName();
	setColor();
	lastAutoReverse = ros::Time::now();
	lastNoise = ros::Time::now();
	srand(time(NULL));
};

GroundRobot::~GroundRobot(){};

void GroundRobot::setStartingPose(){
	switch (robotType){
	case TARGET_ROBOT:
		x = (nRobots/(2*PI))*cos(robotID*(2*PI/nRobots));
		y = (nRobots/(2*PI))*sin(robotID*(2*PI/nRobots));
		z = 0;
		yaw = robotID*2*PI/nRobots;
		break;
	default:
		x = 5*cos(robotID*(2*PI/nRobots));
		y = 5*sin(robotID*(2*PI/nRobots));
		z = 0;
		yaw = atan2(-x, y);
		break;
	}

	refreshTransform();
}

void GroundRobot::setName(){
	std::stringstream ss;
	std::string prefix;
	switch (robotType){
	case TARGET_ROBOT:
		prefix = "trgtRobot";
		break;
	default:
		prefix = "obsRobot";
		break;
	}
	ss << prefix << robotID;
	robotName = ss.str();
}

void GroundRobot::setColor(){
	switch (robotType){
	case TARGET_ROBOT:
		color = robotID%2 ? RED : GREEN;
		break;
	default:
		color = WHITE;
		break;
	}
}

std::string GroundRobot::getName(){
	return robotName;
}

tf::Transform GroundRobot::getTransform(){
	return t;
}

int GroundRobot::getID(){
	return robotID;
}

int GroundRobot::getColor(){
	return color;
}

std::string GroundRobot::getType(){
	switch (robotType){
		case TARGET_ROBOT:
			return "trgtRobot";
			break;
		default:
			return "obsRobot";
			break;
		}
}

int GroundRobot::getTypeID(){
	return robotType;
}

void GroundRobot::advance(ros::Duration cycleTime){
	if (robotType == TARGET_ROBOT) {
		if ((ros::Time::now()-lastAutoReverse).toSec() >= 20.0/simSpeed && !isSpinning) {
			autoReverse();
		}

		if ((ros::Time::now()-lastNoise).toSec() >= 5.0/simSpeed && !isSpinning) {
			noise();
		}

		if (turnAngle) {
			yaw += limitTurn(turnAngle, (PI/2.456) * simSpeed, cycleTime.toSec());
		} else {
			isSpinning = false;
		}

		if (!isSpinning){
			x += 0.33 * simSpeed * cycleTime.toSec() * cos(yaw);
			y += 0.33 * simSpeed * cycleTime.toSec() * sin(yaw);
		}

	} else {
		if (!isStopped){
			yaw = atan2(-x, y);
			x += 0.33 * simSpeed * cycleTime.toSec() * cos(yaw);
			y += 0.33 * simSpeed * cycleTime.toSec() * sin(yaw);
		} else {
			isStopped = false;
		}
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
	switch (robotType){
	case TARGET_ROBOT: if (!isSpinning) reverse();
	break;
	default: isStopped = true;
	break;
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
