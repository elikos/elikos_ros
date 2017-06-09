#include "QuadState.h"

#include "IntersectionTransform.h"

#include <iostream>

namespace localization {

IntersectionTransform::IntersectionTransform(double focalLength, QuadState* state)
    : focalLength_(focalLength), state_(state), pointCloud_(new pcl::PointCloud<pcl::PointXY>())
{
    posePublisher_ = ros::NodeHandle().advertise<geometry_msgs::PoseArray>("localization/features", 5);
    // TODO: Set epsilon for kdtree here maybe ? ...
}


void IntersectionTransform::updateKDTree(const std::vector<Eigen::Vector2f>& imageIntersections)
{
    pointCloud_->clear();
    for (int i = 0; i < imageIntersections.size(); ++i)
    {
        pointCloud_->push_back({ imageIntersections[i].x(), imageIntersections[i].y() });
    }
    kdTree_.setInputCloud(pointCloud_);
}

void IntersectionTransform::transformIntersections(const std::vector<Eigen::Vector2f>& imageIntersections, const ros::Time& stamp)
{
    if (imageIntersections.size() < 1) return;

    updateKDTree(imageIntersections);
    double z = estimateAltitude(imageIntersections);//La caméra pointe vers le bas?

    Eigen::Vector3f smallest(10.0, 10.0, 10.0);

    std::vector<Eigen::Vector3f> transformedIntersections;
    for (int i = 0; i < imageIntersections.size(); ++i)
    {
        Eigen::Vector3f transformedIntersection(0.0, 0.0, z);
        transformIntersectionXY(imageIntersections[i], transformedIntersection);
        if (transformedIntersection.norm() < smallest.norm())
        {
            smallest = transformedIntersection;
        }
        transformedIntersections.push_back(std::move(transformedIntersection));
    }

    std::cout << smallest.x() << ":" << smallest.y() << ":" << smallest.z() << std::endl;
    publishTransformedIntersections(transformedIntersections, stamp);
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
            double height = GRID_SIDE_LENGTH_M * (focalLength_ / std::sqrt(imageDistance));
            double error = std::abs((height - state_->position_.z()) / state_->position_.z());

            // Use current state if error is too high.
            if (error < ALT_ERROR_THRESHOLD)
            {
                //height = state_->position_.z();
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
    double transformCoefficient = std::abs(transformedIntersection.z()) / focalLength_;
    transformedIntersection.x() = (imageIntersection.x() - 320.0) * transformCoefficient;//TODO plz no hardcode
    transformedIntersection.y() = (imageIntersection.y() - 240.0) * transformCoefficient;
}

void IntersectionTransform::publishTransformedIntersections(const std::vector<Eigen::Vector3f>& intersections, const ros::Time& stamp) const
{
    geometry_msgs::PoseArray message = geometry_msgs::PoseArray();
    for (unsigned i = 0; i < intersections.size(); ++i) {
        geometry_msgs::Pose p = geometry_msgs::Pose();
        p.position.x = intersections[i](0);
        p.position.y = intersections[i](1);
        p.position.z = intersections[i](2);
        message.poses.push_back(p);
    }
    message.header.stamp = stamp;
    message.header.frame_id = frameId_;
    posePublisher_.publish(message);
}

}