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

    inline void setIsUpdated(bool isUpdated);

    inline void setPose(const tf::Pose& pose);
    inline const tf::Pose& getPose() const;
    inline const tf::Vector3& getDirection()const;
    inline const void setDirection(const tf::Vector3& direction);

    void updatePositionRadius(const double& dt);
    tfScalar getDistance(AbstractRobot* robot);

protected:
    bool isUpdated_{ false };
    double positionRadius_{ 30.0 };
    tf::Pose pose_;
    tf::Vector3 direction_;
};

inline const tf::Pose& AbstractRobot::getPose() const
{
   return pose_;
}


inline void AbstractRobot::setPose(const tf::Pose& pose)
{
    direction_ = tf::Vector3(pose.getOrigin() - pose_.getOrigin()).normalized();
    pose_ = pose;
}


inline void AbstractRobot::setIsUpdated(bool isUpdated)
{
    isUpdated_ = isUpdated;
}

inline const tf::Vector3& AbstractRobot::getDirection() const
{
    return direction_;
}

inline const void AbstractRobot::setDirection(const tf::Vector3& direction)
{
    direction_ = direction;
}

}

#endif /// AI_ROBOT_H
