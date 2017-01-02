#ifndef DETECTED_INTERSECTION_H
#define DETECTED_INTERSECTION_H

#include <Eigen/Core>

namespace localization
{

class DetectedIntersection
{
public:

    DetectedIntersection(const Eigen::Vector2f& imagePosition);

private:

    Eigen::Vector2f imagePosition_;

};

}

#endif // DETECTED_INTERSECTION_H