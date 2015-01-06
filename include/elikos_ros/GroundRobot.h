/*
 * GroundRobot.h
 *
 *  Created on: Jan 3, 2015
 *      Author: Antonio Sanniravong
 */

#ifndef __GROUNDROBOT_H_
#define __GROUNDROBOT_H_

#define PI 3.14159265
enum robot_type {
	TARGET_ROBOT,
	OBSTACLE_ROBOT_RND,
	OBSTACLE_ROBOT_100CM,
	OBSTACLE_ROBOT_125CM,
	OBSTACLE_ROBOT_150CM,
	OBSTACLE_ROBOT_175CM,
	OBSTACLE_ROBOT_200CM
};

class GroundRobot {
public:
	GroundRobot(robot_type type, int nbRobots, int id, double simulationSpeed);
	~GroundRobot();
	void advance(ros::Duration cycleTime);
	void collide();
	void touch();
	std::string getName();
	tf::Transform getTransform();
private:
	int robotID, robotType, nRobots;
	std::string robotName;
	bool isSpinning, isStopped;
	double x, y, z, yaw, turnAngle, simSpeed;
	tf::Transform t;
	tf::Vector3 v;
	tf::Quaternion q;
	ros::Time lastNoise, lastAutoReverse;
	void reverse();
	void autoReverse();
	void noise();
	void setStartingPose();
	void setName();
	void refreshTransform();
	double limitTurn(double& angle, double angularSpeed, double cycleDuration);
};

#endif /* __GROUNDROBOT_H_ */
