#include "Robot.hpp"

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