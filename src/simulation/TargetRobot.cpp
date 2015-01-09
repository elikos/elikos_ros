#include <ros/ros.h>
#include "TargetRobot.hpp"

TargetRobot::TargetRobot(int id) : Robot(id){

}

tf::Transform TargetRobot::getTransform() {
    return this->transform;
}

visualization_msgs::Marker getVizMarket(){
    
}
