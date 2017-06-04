//
// Created by olivier on 30/06/16.
//

#include "Line.h"

#include "Segment.h"

namespace util
{

Segment::Segment(const tf::Point& A, const tf::Point& B)
    : A_(A), B_(B)
{
}

bool Segment::getIntersectionPoint(const Line& line, tf::Point& intersection) const
{
    return line.getIntersectionPoint(*this, intersection);
}

bool Segment::isIntersecting(const Line& line) const
{
    return line.isIntersecting(*this);
}

}
