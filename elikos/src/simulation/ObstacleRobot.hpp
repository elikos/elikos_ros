#ifndef SIM_OBSTACLE_ROBOT_HPP
#define SIM_OBSTACLE_ROBOT_HPP

#include "Robot.hpp"

namespace elikos_sim {

/**
* Class representing obstacle robots
*/
class ObstacleRobot : public Robot{
public:
    ObstacleRobot(int id, int numRobots, double simulationSpeed);
    ~ObstacleRobot();

    visualization_msgs::Marker getVizMarker();

    void collide();
    void move(ros::Duration cycleTime);

private:
    bool isStopped;
};

} // namespace elikos_sim

#endif // SIM_OBSTACLE_ROBOT_HPP
