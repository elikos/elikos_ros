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
	ID = id;
	simSpeed = simulationSpeed;
	setStartingPose();
	setName();
	/*
	    //! @brief 		Init function
        //! @details   	The parameters specified here are those for for which we can't set up
        //!    			reliable defaults, so we need to have the user set them.
        Pid(
                dataType kp,
                dataType ki,
                dataType kd,
                ControllerDirection controllerDir,
                OutputMode outputMode,
                double samplePeriodMs,
                dataType minOutput,
                dataType maxOutput,
                dataType setPoint);
	 */
	Vel_X_PID = new Pid<double>(0.0, 0.0, 0.0, Pid<double>::PID_DIRECT, Pid<double>::DONT_ACCUMULATE_OUTPUT, 33.0, 0.0, 0.0, 0.0);
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
