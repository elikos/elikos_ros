#ifndef AI_ROBOT_H
#define AI_ROBOT_H

#include <tf/tf.h>

namespace ai
{

class AbstractRobot
{
public:
    static const double SPEED;

    AbstractRobot() = default;
    virtual ~AbstractRobot() = 0;

    inline const tf::Pose& getPose() const;

    inline void setPose(const tf::Pose pose);
    inline void setIsUpdated(const bool& isUpdated);

    void updatePositionRadius(const double& dt);
    tfScalar getDistance(AbstractRobot* robot);


private:
    bool isUpdated_{ false };
    double positionRadius_{ 30.0 };
    tf::Pose pose_;
};

inline const tf::Pose& AbstractRobot::getPose() const
{
   return pose_;
}


inline void AbstractRobot::setPose(const tf::Pose pose)
{
   pose_ = pose;
}


inline void AbstractRobot::setIsUpdated(const bool& isUpdated)
{
    isUpdated_ = isUpdated;
}

};

#endif /// AI_ROBOT_H
