#ifndef TARGET_ROBOT_H
#define TARGET_ROBOT_H

#include <tf/tf.h>
#include "Robot.hpp"

namespace elikos_sim {

/**
* Class representing target robots in the arena
*/
class TargetRobot : public Robot{
public:
    enum Color{
        RED,
        GREEN
    };

    TargetRobot(int id, int numRobots, double simulationSpeed);

    visualization_msgs::Marker getVizMarker();
    Color getColor() { return this->color; }

    void move(ros::Duration cycleTime);
    void collide();
private:
    Color color;
    ros::Time lastNoise, lastAutoReverse;
    bool isSpinning, isStopped;

    void reverse();
    void autoReverse();
    void noise();
    double limitTurn(double& angle, double angularSpeed, double cycleDuration);
};

} // namespace elikos_sim

#endif /* TARGET_ROBOT_H */
