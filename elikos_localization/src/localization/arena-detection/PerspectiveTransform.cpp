
#include "PerspectiveTransform.h"
#include "Line.h"
#include "LineGroup.h"
#include "RANSAC.h"
#include "Eigen/Core"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cmath>

namespace localization
{

using Vector = Eigen::Vector2f;

Eigen::Vector2f PerspectiveTransform::translate(const Eigen::Vector2f& v, const Eigen::Vector2f& translation)
{
    return v + translation;
}

Eigen::Vector2f PerspectiveTransform::rotate(const Eigen::Vector2f& v, double theta)
{
    double cosTheta = cosf(theta);
    double sinTheta = sinf(theta);

    Eigen::Vector2f rotated;
    rotated.x() = v.x() * cosTheta - v.y() * sinTheta;
    rotated.y() = v.x() * sinTheta + v.y() * cosTheta;

    return rotated;
}

}