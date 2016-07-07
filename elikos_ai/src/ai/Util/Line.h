#ifndef UTIL_LINE_H
#define UTIL_LINE_H

#include <tf/tf.h>

namespace util
{

class Segment;

class Line
{
public:
    Line(tf::Point point, tf::Vector3 orientation);
    ~Line() = default;

    inline void setOrigin(const tf::Point origin);
    inline const tf::Point& getOrigin() const;

    inline void setOrientation(const tf::Vector3 orientation);
    inline const tf::Vector3 getOrientation() const;

    tf::Vector3 get2DPerpendicularOrientation() const;

    bool getIntersectionPoint(const Segment& segment, tf::Point& intersection) const;

    bool isIntersecting(const Segment& segment) const;

private:
    tf::Point origin_;
    tf::Vector3 orientation_;


    bool projectionIsIntersecting(const tf::Vector3& orientation, const tf::Point& dA, const tf::Point& dB) const;

    Line() = delete;
};

inline void Line::setOrigin(const tf::Point origin)
{
    origin_ = origin;
}

inline const tf::Point& Line::getOrigin() const
{
    return origin_;
}

inline void Line::setOrientation(const tf::Vector3 orientation)
{
    orientation_ = orientation.normalized();
}

inline const tf::Vector3 Line::getOrientation() const
{
    return orientation_;
}


}

#endif // UTIL_LINE_H
