//
// Created by olivier on 30/06/16.
//

#include "Segment.h"

#include "Line.h"

namespace util
{

Line::Line(tf::Point origin, tf::Vector3 orientation)
    : origin_(origin), orientation_(orientation.normalized())
{
}

bool Line::getIntersectionPoint(const Segment& segment, tf::Point& intersection) const
{
    if (!isIntersecting(segment)) return false;

    //TODO: more math stuff here...
}

bool Line::isIntersecting(const Segment& segment) const
{
    tf::Vector3 dA = segment.getA() - origin_;
    tf::Vector3 dB = segment.getB() - origin_;

    tf::Vector3 perpendicular = get2DPerpendicularOrientation();

    return projectionIsIntersecting(orientation_, dA, dB) &&
           projectionIsIntersecting(perpendicular, dA, dB);
}

tf::Vector3 Line::get2DPerpendicularOrientation() const
{
    tf::Vector3 perpendicular( orientation_.getY(),
                              -orientation_.getX(),
                               orientation_.getZ());
    return perpendicular.normalized();
}

bool Line::projectionIsIntersecting(const tf::Vector3& orientation, const tf::Point& dA, const tf::Point& dB) const
{
    double projA = orientation.dot(dA);
    double projB = orientation.dot(dB);
    return std::signbit(projA) != std::signbit(projB);
}

}
