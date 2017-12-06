//
// Created by olivier on 30/06/16.
//

#include "Segment.h"

#include "Line.h"

namespace util
{

Line::Line(tf::Point origin, tf::Vector3 orientation)
    : origin_(origin)
{
    orientation.setZ(0.0);
    orientation_ = orientation.normalized();
}

bool Line::getIntersectionPoint(const Segment& segment, tf::Point& intersection) const
{
    if (!isIntersecting(segment))
    {
        return false;
    }

    tf::Point A1 = segment.getA();
    tf::Point B1 = segment.getB();
    double dy1 = segment.getB().y() - segment.getA().y();
    double dx1 = segment.getA().x() - segment.getB().x();

    tf::Point A2 = origin_;
    tf::Point B2 = origin_ + orientation_;
    double dy2 = B2.y() - A2.y();
    double dx2 = A2.x() - B2.x();

    double det = dy1 * dx2 - dy2 * dx1;
    if (det != 0.0) {
        double C1 = dy1 * A1.x() + dx1 * B1.y();
        double C2 = dy2 * A2.x() + dx2 * B2.y();

        intersection.setX((dx2 * C1 - dx1 * C2) / det);
        intersection.setY((dy1 * C2 - dy2 * C1) / det);

    } else {
        return false;
    }
    return true;
}

bool Line::isIntersecting(const Segment& segment) const
{
    tf::Vector3 originDistanceA = segment.getA() - origin_;
    tf::Vector3 originDistanceB = segment.getB() - origin_;
    tf::Vector3 perpendicular = get2DPerpendicularOrientation();
    return projectionIsIntersecting(perpendicular, originDistanceA, originDistanceB);
}

bool Line::projectionIsIntersecting(const tf::Vector3& orientation, const tf::Point& originDistanceA, const tf::Point& originDistanceB) const
{
    double projA = orientation.dot(originDistanceA);
    double projB = orientation.dot(originDistanceB);
    return std::signbit(projA) && !std::signbit(projB);
}

tf::Vector3 Line::get2DPerpendicularOrientation() const
{
    tf::Vector3 perpendicular(orientation_.getY(),
                             -orientation_.getX(),
                              orientation_.getZ());
    return perpendicular.normalized();
}

}
