#ifndef AI_ROBOT_H
#define AI_ROBOT_H

#include <tf/tf.h>

namespace ai
{

class Robot
{
public:
    static const double SPEED;

    Robot() = default;
    virtual ~Robot() = 0;

    inline const tf::Pose& getPose() const;

    inline void setPose(const tf::Pose pose);
    inline void setIsUpdated(const bool& isUpdated);

    void updatePositionRadius(const double& dt);
    tfScalar getDistance(Robot* robot);


private:
    bool isUpdated_{ false };
    double positionRadius_{ 30.0 };
    tf::Pose pose_;
};

inline const tf::Pose& Robot::getPose() const
{
   return pose_;
}


inline void Robot::setPose(const tf::Pose pose)
{
   pose_ = pose;
}


inline void Robot::setIsUpdated(const bool& isUpdated)
{
    isUpdated_ = isUpdated;
}

};

#endif /// AI_ROBOT_H
