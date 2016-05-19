#ifndef AI_OBSTACLEROBOT_H
#define AI_OBSTACLEROBOT_H

#include "Robot.h"

namespace ai
{

class ObstacleRobot : public Robot
{
public:
    ObstacleRobot(const int& id);
    ObstacleRobot()=default;
    ~ObstacleRobot()=default;
    inline int getId() const;

private:
    int id_;
};

inline int ObstacleRobot::getId() const
{
    return id_;
}

}

#endif /// AI_OBSTACLEROBOT_H
