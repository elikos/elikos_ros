#ifndef AI_ROBOT_H
#define AI_ROBOT_H

#include <tf/tf.h>

namespace ai
{

class Robot
{
public:
    Robot() = default;
    Robot(const int& id);
    virtual ~Robot() = 0;

    inline int getId() const;
    inline const tf::Vector3& getPosition() const;
    inline const tf::Quaternion& getOrientation() const;

    inline void setPosition(const tf::Vector3 position);
    inline void setOrientation(const tf::Quaternion orientation);

private:
    int id_;
    tf::Vector3 position_;
    tf::Quaternion orientation_;
};

inline int Robot::getId() const
{
   return id_;
}

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

};

#endif /// AI_ROBOT_H
