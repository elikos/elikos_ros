#include "QuadState.h"

#include <Eigen/Core>

#include "IntersectionTransform.h"

namespace localization {

IntersectionTransform::IntersectionTransform(double focalLength, QuadState* state)
    : focalLength_(focalLength), state_(state)
{
}

void IntersectionTransform::transformIntersections(const std::vector<Eigen::Vector2f>& imageIntersections) const
{
    if (imageIntersections.size() < 1) return;
    std::vector<Eigen::Vector3f> transformedIntersections;
    for (int i = 0; i < imageIntersections.size(); ++i)
    {
        transformedIntersections.push_back(std::move(transformIntersection(imageIntersections[i])));
    }
    publishTransformedIntersections(transformedIntersections);
}

double IntersectionTransform::estimateAltitude(const std::Vector<Eigen::Vector2f>& imageIntersections) const
{
    double height = 0.0;
    if (imageIntersections.size() > 1) 
    {
    }
    // We detected only 1 intersection, we use the current state of the quad.
    else 
    {
        height = state_->position_.z();
    }
    return height;
}

Eigen::Vector3f IntersectionTransform::transformIntersection(const Eigen::Vector2f& imageIntersection) const
{

}

void IntersectionTransform::publishTransformedIntersections(const std::vector<Eigen::Vector3f>& intersections) const
{
}

}