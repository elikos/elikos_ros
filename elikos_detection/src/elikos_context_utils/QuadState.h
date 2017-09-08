#ifndef QUAD_STATE_H
#define QUAD_STATE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "tf/tf.h"

class QuadState
{
public:
    QuadState() = default;
    QuadState(const std::string& cameraFrame);
    ~QuadState() = default;

    inline void setCameraFrame(const std::string& cameraFrame);

    bool update(ros::Time stamp);

    ros::Time getTimeStamp() const;
    inline tf::StampedTransform getOrigin2Fcu() const;
    inline tf::StampedTransform getFcu2Camera() const;
    inline tf::StampedTransform getOrigin2Attitude() const;

private:
    std::string cameraFrame_;

    tf::TransformListener tfListener_;
    tf::TransformBroadcaster tfBroadcaster_;

    tf::StampedTransform origin2fcu_;
    tf::StampedTransform fcu2camera_;
    tf::StampedTransform origin2attitude_;
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

inline tf::StampedTransform QuadState::getOrigin2Attitude() const 
{
    return origin2attitude_;
}

inline void QuadState::setCameraFrame(const std::string& cameraFrame)
{
    cameraFrame_ = cameraFrame;
}

#endif // QUAD_STATE_H
