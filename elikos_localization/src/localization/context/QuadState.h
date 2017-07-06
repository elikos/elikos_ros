#ifndef QUAD_STATE_H
#define QUAD_STATE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>

#include "tf/tf.h"

namespace localization
{

class QuadState
{
public:
    QuadState(const std::string& cameraFrame);
    ~QuadState() = default;

    void update(ros::Time stamp);

    ros::Time getTimeStamp() const;
    tf::StampedTransform getOrigin2Fcu() const;
    tf::StampedTransform getFcu2Camera() const;

private:
    const std::string cameraFrame_;

    tf::TransformListener tfListener_;

    tf::StampedTransform origin2fcu_;
    tf::StampedTransform fcu2camera_;
    ros::Time timeStamp_;
};

inline ros::Time QuadState::getTimeStamp() const
{
    return timeStamp_;
}

inline tf::StampedTransform QuadState::getOrigin2Fcu() const
{
    return origin2fcu_;
}

inline tf::StampedTransform QuadState::getFcu2Camera() const
{
    return fcu2camera_;
}


}

#endif // QUAD_STATE_H