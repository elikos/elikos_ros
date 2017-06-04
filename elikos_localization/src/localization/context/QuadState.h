#ifndef QUAD_STATE_H
#define QUAD_STATE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace localization
{

class QuadState
{
public:

    static QuadState* getInstance();
    static void freeInstance();

    Eigen::Vector3f position_;
    Eigen::Vector3f velocity_;
    Eigen::Vector3f linearAcceleration_;

    Eigen::Matrix3f positionCovariance_;
    Eigen::Matrix3f velocityCovariance_;
    Eigen::Matrix3f linearAccelerationCovariance_;

    Eigen::Quaternionf orientation_;
    Eigen::Vector3f angularVelocity_;

    Eigen::Matrix3f orientationCovariance_;
    Eigen::Matrix3f angularVelocityCovariance_;

private:

    static QuadState* instance_;

    QuadState() = default;
    ~QuadState() = default;
};

}

#endif // QUAD_STATE_H