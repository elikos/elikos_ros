#ifndef ROBOT_H
#define ROBOT_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <string>
#include <visualization_msgs/MarkerArray.h>

namespace elikos_sim {

/**
* Abstract base class representing robots in the arena (obstacles and targets)
*/
class Robot{
public:
    std::string getName() const { return this->name; }
    int getID() { return this->id; }
    tf::Transform getTransform() const { return this->transform; }
    virtual visualization_msgs::Marker getVizMarker() = 0;

    virtual void collide() = 0;
    virtual void move(ros::Duration cycleTime) = 0;

protected:
    Robot(int id, double simSpeed);
    ~Robot() { }
    void refreshTransform();

    int id;
    std::string name;
    double x, y, z, yaw, turnAngle, simSpeed;
    tf::Transform transform;
    tf::Vector3 v;
    tf::Quaternion q;
private:
    Robot();
};

} // namespace elikos_sim

#endif /* GROUNDROBOT_H */
