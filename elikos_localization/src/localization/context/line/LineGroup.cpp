//
// Created by olivier on 10/5/16.
//

#include "LineGroup.h"

#include "Line.h"

namespace localization
{

LineGroup::LineGroup(const Line& line)
    : avgOrientation_(line.getOrientation()),
      avgRho_(line.getRho()),
      avgCentroid_(line.getCentroid())
{
    lines_.push_back(&line);
}

void LineGroup::add(const Line& line)
{
    Eigen::Vector2f orientation = line.getOrientation();
    float product = orientation.dot(avgOrientation_);
    if (product < 0)
    {
        orientation = -orientation;
    }

    avgOrientation_ *= (double)(lines_.size());
    avgRho_ *= (double)(lines_.size());
    avgCentroid_ *= (double)(lines_.size());

    lines_.push_back(&line);

    avgOrientation_ += orientation;
    avgRho_ += line.getRho();
    avgCentroid_ += line.getCentroid();

    avgOrientation_ /= (double)(lines_.size());
    avgRho_ /= (double)(lines_.size());
    avgCentroid_ /= (double)(lines_.size());

}

bool LineGroup::isCollateral(const Line& line, double threshold)
{
    return std::abs(line.getOrientation().dot(avgOrientation_)) > threshold;
}

Line LineGroup::convertToLine() const
{
    return Line(avgCentroid_, avgOrientation_);
}

}
