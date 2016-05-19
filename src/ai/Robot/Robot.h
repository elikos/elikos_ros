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

    inline const tf::Vector3& getPosition() const;
    inline const tf::Quaternion& getOrientation() const;

    inline void setPosition(const tf::Vector3 position);
    inline void setOrientation(const tf::Quaternion orientation);
    inline void setIsUpdated(const bool& isUpdated);

    void updatePositionRadius(const double& dt);


private:
    bool isUpdated_{ false };
    double positionRadius_{ 30.0 };
    tf::Vector3 position_;
    tf::Quaternion orientation_;
};

inline const tf::Vector3& Robot::getPosition() const
{
   return position_;
}

inline const tf::Quaternion& Robot::getOrientation() const
{
   return orientation_;
}

inline void Robot::setPosition(const tf::Vector3 position)
{
   position_ = position;
}

inline void Robot::setOrientation(const tf::Quaternion orientation)
{
   orientation_ = orientation;
}

inline void Robot::setIsUpdated(const bool& isUpdated)
{
    isUpdated_ = isUpdated;
}

};

#endif /// AI_ROBOT_H
