#ifndef ROBOT_H
#define ROBOT_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <string>
#include <visualization_msgs/MarkerArray.h>

/**
* Abstract base class representing robots in the arena (obstacles and targets)
*/
class Robot{
public:
    Robot(int id) : id(id) { }
    ~Robot();

    std::string getName() const { return this->name; }
    int getID() { return this->id; }
    virtual tf::Transform getTransform() = 0;
    virtual visualization_msgs::Marker getVizMarker() = 0;

    virtual void collide() = 0;
    virtual void move(ros::Duration cycleTime) = 0;
private:
    Robot();

    int id;
    std::string name;
};

#endif /* GROUNDROBOT_H */