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
	void advance(ros::Duration cycleTime);
	void collide();
	std::string getName();
	tf::Transform getTransform();
	int getID();
	void setPosPID();
	void setVelPID();
	void setPosTarget();
	void setVelTarget();
private:
	int ID;
	std::string Name;
	bool isLanded;
	double x, y, z, yaw, simSpeed;
	double x_sp, y_sp, z_sp, yaw_sp;
	tf::Transform t;
	tf::Vector3 v;
	tf::Quaternion q;
	void setStartingPose();
	void setName();
	void refreshTransform();
	double limitTurn(double& angle, double angularSpeed, double cycleDuration);
	Pid<double>* Vel_X_PID;

};

#endif /* __MAV_H_ */
