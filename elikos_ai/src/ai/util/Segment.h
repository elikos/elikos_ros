//
// Created by olivier on 30/06/16.
//

#ifndef UTIL_SEGMENT_H
#define UTIL_SEGMENT_H

#include <tf/tf.h>

namespace util
{

class Line;

class Segment
{
public:
    Segment(const tf::Point& A, const tf::Point& B);
    virtual ~Segment() = default;

    inline void setA(tf::Point A);
    inline const tf::Point& getA() const;

    inline void setB(tf::Point B);
    inline const tf::Point& getB() const;

    bool getIntersectionPoint(const Line& line, tf::Point& intersection) const;

    bool isIntersecting(const Line& line) const;

private:
    tf::Point A_;
    tf::Point B_;

    Segment() = delete;
};

inline void Segment::setA(tf::Point A)
{
    A_ = A;
}

inline const tf::Point& Segment::getA() const
{
    return A_;
}

inline void Segment::setB(tf::Point B)
{
    B_ = B;
}

inline const tf::Point& Segment::getB() const
{
    return B_;
}

}

#endif // UTIL_SEGMENT_H
