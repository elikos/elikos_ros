/*
 * GroundRobot.h
 *
 *  Created on: Jan 3, 2015
 *      Author: Antonio Sanniravong
 */

#ifndef __GROUNDROBOT_H_
#define __GROUNDROBOT_H_

#define PI 3.14159265

class GroundRobot {
public:
	GroundRobot(int id);
	~GroundRobot();
	void advance(ros::Duration cycleTime);
	void collide();
	void touch();
	std::string getName();
	tf::Transform getTransform();
private:
	int robotID;
	std::string robotName;
	bool isSpinning;
	double x, y, z, yaw, turnAngle;
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
