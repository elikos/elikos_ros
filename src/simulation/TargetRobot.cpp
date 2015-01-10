#include <ros/ros.h>
#include "TargetRobot.hpp"

#ifndef PI
#define PI 3.14159265
#endif
#define ROBOT_TYPE "trgtRobot"

TargetRobot::TargetRobot(int id, int numRobots, double simulationSpeed) : Robot(id, simulationSpeed) {
    //Starting pose
    x = (numRobots / (2 * PI)) * cos(id * (2 * PI / numRobots));
    y = (numRobots / (2 * PI)) * sin(id * (2 * PI / numRobots));
    z = 0;
    yaw = id * 2 * PI / numRobots;

    char buf[15];
    sprintf(buf, "trgtRobot%d", id);
    this->name = buf;
    this->isSpinning = false;
    this->isStopped = false;
    this->color = id%2 ? RED : GREEN;

    lastAutoReverse = ros::Time::now();
    lastNoise = ros::Time::now();
    refreshTransform();
}

void TargetRobot::move(ros::Duration cycleTime) {
    if ((ros::Time::now() - lastAutoReverse).toSec() >= 20.0 / simSpeed && !isSpinning) {
        autoReverse();
    }

    if ((ros::Time::now() - lastNoise).toSec() >= 5.0 / simSpeed && !isSpinning) {
        noise();
    }

    if (!isSpinning) {
        x += 0.33 * simSpeed * cycleTime.toSec() * cos(yaw);
        y += 0.33 * simSpeed * cycleTime.toSec() * sin(yaw);
    }

    if (turnAngle) {
        yaw += limitTurn(turnAngle, (PI / 2.456) * simSpeed, cycleTime.toSec());
    } else {
        isSpinning = false;
    }

    this->refreshTransform();
}

void TargetRobot::reverse() {
    turnAngle -= PI; // 180 Degrees clockwise
    isSpinning = true;
}

void TargetRobot::autoReverse() {
    lastAutoReverse = ros::Time::now();
    reverse();
}

void TargetRobot::noise() {
    lastNoise = ros::Time::now();
    turnAngle += (PI / 9) * (rand() % 1000 - 500) / 500; // Max 20 degree deviation
}

void TargetRobot::collide(){
    if(!isSpinning)
        reverse();
}

double TargetRobot::limitTurn(double& angle, double angularSpeed, double cycleDuration){
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

visualization_msgs::Marker TargetRobot::getVizMarker() {
    visualization_msgs::Marker marker;
    float r = 0.0, g = 0.0, b = 0.0;

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = this->name;
    marker.id = this->id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.35;
    marker.scale.y = 0.35;
    marker.scale.z = 0.1;
    marker.pose.position.x = transform.getOrigin().getX();
    marker.pose.position.y = transform.getOrigin().getY();
    marker.pose.position.z = transform.getOrigin().getZ() + marker.scale.z / 2;
    marker.pose.orientation.x = transform.getRotation().getX();
    marker.pose.orientation.y = transform.getRotation().getY();
    marker.pose.orientation.z = transform.getRotation().getZ();
    marker.pose.orientation.w = transform.getRotation().getW();

    if (color == RED) {
        r = 1.0;
    } else {
        g = 1.0;
    }
    if(id == 1){
        b = 1.0;
    }
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    return marker;
}
