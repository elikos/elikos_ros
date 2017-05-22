#include "QuadState.h"

#include "IntersectionTransform.h"

namespace localization {

IntersectionTransform::IntersectionTransform(double focalLength, QuadState* state)
    : focalLength_(focalLength), state_(state), pointCloud_(new pcl::PointCloud<pcl::PointXY>())
{
    // TODO: Set epsilon for kdtree here maybe ? ...
}


void IntersectionTransform::updateKDTree(const std::vector<Eigen::Vector2f>& imageIntersections)
{
    pointCloud_->clear();
    for (int i = 0; i < imageIntersections.size(); ++i)
    {
        pointCloud_->push_back({ imageIntersections[i].x(), imageIntersections[i].z() });
    }
    kdTree_.setInputCloud(pointCloud_);
}

void IntersectionTransform::transformIntersections(const std::vector<Eigen::Vector2f>& imageIntersections)
{
    if (imageIntersections.size() < 1) return;

    updateKDTree(imageIntersections);
    double z = estimateAltitude(imageIntersections);

    std::vector<Eigen::Vector3f> transformedIntersections;
    for (int i = 0; i < imageIntersections.size(); ++i)
    {
        Eigen::Vector3f transformedIntersection(0.0, 0.0, z);
        transformIntersectionXY(imageIntersections[i], transformedIntersection);
        transformedIntersections.push_back(std::move(transformedIntersection));
    }
    publishTransformedIntersections(transformedIntersections);
}

double IntersectionTransform::estimateAltitude(const std::vector<Eigen::Vector2f>& imageIntersections)
{
    double estimate = 0.0;
    double totalHeight = 0.0;
    int sampleSize = 0;
    if (imageIntersections.size() > 1) 
    {
        std::vector<int> indices(2);
        std::vector<float> distances(2);
        for (int i = 0; i < imageIntersections.size(); ++i)
        {
            kdTree_.nearestKSearch(pointCloud_->at(i), 2, indices, distances);
            // Take the greatest since the smallest is the same point
            double imageDistance = (distances[0] > distances[1]) ? distances[0] : distances[1];

            // Compute local height estimate and error.
            double height = (GRID_SIDE_LENGTH_M / focalLength_) * imageDistance;
            double error = std::abs((height - state_->position_.z()) / state_->position_.z());

            // Use current state if error is too high.
            if (error < ALT_ERROR_THRESHOLD)
            {
                height = state_->position_.z();
            }
            totalHeight += height;
            ++sampleSize;
        }
        // return average of local height estimates.
        estimate = totalHeight / sampleSize;
    }
    // We detected only 1 intersection, we use the current state of the quad.
    else 
    {
        estimate = state_->position_.z();
    }
    return estimate;
}

void IntersectionTransform::transformIntersectionXY(const Eigen::Vector2f& imageIntersection, 
                                                    Eigen::Vector3f& transformedIntersection) const
{
    double transformCoefficient = transformedIntersection.z() / focalLength_;
    transformedIntersection.x() = imageIntersection.x() * transformCoefficient;
    transformedIntersection.y() = imageIntersection.y() * transformCoefficient;

    // TODO: Add offset from camera angle.
}

void IntersectionTransform::publishTransformedIntersections(const std::vector<Eigen::Vector3f>& intersections) const
{
}

}