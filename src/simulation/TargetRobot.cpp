#include <ros/ros.h>
#include "TargetRobot.hpp"

#ifndef PI
#define PI 3.14159265
#endif

TargetRobot::TargetRobot(int id, int numRobots) : Robot(id){
    //Starting pose
    x = (numRobots/(2*PI))*cos(id*(2*PI/numRobots));
    y = (numRobots/(2*PI))*sin(id*(2*PI/numRobots));
    z = 0;
    yaw = id*2*PI/numRobots;
}

tf::Transform TargetRobot::getTransform() {
    return this->transform;
}

visualization_msgs::Marker getVizMarket(){

}
