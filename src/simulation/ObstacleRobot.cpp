#include "ObstacleRobot.hpp"

#ifndef PI
#define PI 3.14159265
#endif

ObstacleRobot::ObstacleRobot(int id, int numRobots, double simulationSpeed) : Robot(id, simulationSpeed) {
    //starting pose
    x = 5 * cos(id * (2 * PI / numRobots));
    y = 5 * sin(id * (2 * PI / numRobots));
    z = 0;
    yaw = atan2(-x, y);
    this->isStopped = false;
    this->simSpeed = simulationSpeed;
    char buf[15];
    sprintf(buf, "obsRobot%d", id);
    this->name = buf;
    refreshTransform();
}

void ObstacleRobot::collide() {
    isStopped=true;
}

void ObstacleRobot::move(ros::Duration cycleTime) {
    if (!isStopped){
        x += 0.33 *  cycleTime.toSec() * cos(yaw);
        y += 0.33 *  cycleTime.toSec() * sin(yaw);
        yaw = atan2(-x, y) - (0.33 *  cycleTime.toSec())/10;
    } else {
        isStopped = false;
    }
    this->refreshTransform();
}

visualization_msgs::Marker ObstacleRobot::getVizMarker() {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = this->name;
    marker.id = this->id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.35;
    marker.scale.y = 0.35;
    marker.scale.z = 1.5;
    marker.pose.position.x = transform.getOrigin().getX();
    marker.pose.position.y = transform.getOrigin().getY();
    marker.pose.position.z = transform.getOrigin().getZ() + marker.scale.z / 2;
    marker.pose.orientation.x = transform.getRotation().getX();
    marker.pose.orientation.y = transform.getRotation().getY();
    marker.pose.orientation.z = transform.getRotation().getZ();
    marker.pose.orientation.w = transform.getRotation().getW();

    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    return marker;
}