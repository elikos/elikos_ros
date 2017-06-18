#ifndef QUAD_STATE_H
#define QUAD_STATE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "tf/tf.h"

namespace localization
{

class QuadState
{
public:

    QuadState() = default;
    ~QuadState() = default;

    tf::StampedTransform origin2fcu;
    tf::StampedTransform fcu2camera;

};

}

#endif // QUAD_STATE_H