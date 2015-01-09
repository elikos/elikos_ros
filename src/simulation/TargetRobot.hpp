#ifndef TARGET_ROBOT_H
#define TARGET_ROBOT_H

#include <tf/tf.h>
#include "Robot.hpp"
/**
* Class representing target robots in the arena
*/
class TargetRobot : public Robot{
public:
    enum Color{
        RED,
        GREEN
    };

    TargetRobot(int id);
    ~TargetRobot() {
    }

    tf::Transform getTransform();
    visualization_msgs::Marker getVizMarker();

private:
    tf::Transform transform;
};

#endif /* TARGET_ROBOT_H */