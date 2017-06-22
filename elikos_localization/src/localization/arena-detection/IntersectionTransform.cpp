#include <iostream>

#include "tf/tf.h"
#include <geometry_msgs/Point.h>

#include "QuadState.h"

#include "IntersectionTransform.h"

namespace localization {

IntersectionTransform::IntersectionTransform(const CameraInfo& cameraInfo, const QuadState& state)
    : cameraInfo_(cameraInfo), state_(state), pointCloud_(new pcl::PointCloud<pcl::PointXY>())
{
    // TODO: Set epsilon for kdtree here maybe ? ...
    ros::NodeHandle nh;

    intersectionPub_ = nh.advertise<elikos_ros::IntersectionArray>("intersections", 1);
    debugPub_ = nh.advertise<visualization_msgs::MarkerArray>("intersection_debug", 1);

    marker_.header.frame_id = "elikos_fcu";
    marker_.header.stamp = ros::Time::now();

    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::SPHERE;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose.position.x = 0;
    marker_.pose.position.y = 0;
    marker_.pose.position.z = 0;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;
    marker_.scale.x = 0.05;
    marker_.scale.y = 0.05;
    marker_.scale.z = 0.05;
    marker_.color.a = 1.0; // Don't forget to set the alpha!
    marker_.color.r = 0.0;
    marker_.color.g = 1.0;
    marker_.color.b = 0.0;


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

void IntersectionTransform::transformIntersections(const std::vector<Eigen::Vector2f>& imageIntersections, const ros::Time stamp)
{
    if (imageIntersections.size() < 1) return;

    updateKDTree(imageIntersections);
    double z = estimateAltitude(imageIntersections);

    tf::Vector3 cameraDirection(0.0, 0.0, 1.0);
    cameraDirection =  state_.origin2fcu * state_.fcu2camera * cameraDirection;

    tf::Vector3 height(0.0, 0.0, z);

    tf::Vector3 offset = (cameraDirection * height.length()) / cameraDirection.dot(tf::Vector3(0.0, 0.0, -1.0)) + height;

    tf::Vector3 smallest(10.0, 10.0, 10.0);

    std::vector<tf::Vector3> transformedIntersections;
    for (int i = 0; i < imageIntersections.size(); ++i)
    {
        tf::Vector3 transformedIntersection(0.0, 0.0, z);
        transformIntersectionXY(imageIntersections[i], transformedIntersection, offset);

        if (transformedIntersection.length() < smallest.length())
        {
            smallest = transformedIntersection;
        }
        transformedIntersections.push_back(std::move(transformedIntersection));
    }

    publishTransformedIntersections(imageIntersections, transformedIntersections, stamp);
}

double IntersectionTransform::estimateAltitude(const std::vector<Eigen::Vector2f>& imageIntersections)
{
    double estimate = 0.0;
    double totalHeight = 0.0;
    int sampleSize = 0;

    const tf::Vector3& currentPosition = state_.origin2fcu.getOrigin();

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
            double height = GRID_SIDE_LENGTH_M * (cameraInfo_.focalLength / std::sqrt(imageDistance));
            double error = std::abs((height - currentPosition.z()) / currentPosition.z());

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
        estimate = currentPosition.z();
    }
    return estimate;
}

void IntersectionTransform::transformIntersectionXY(const Eigen::Vector2f& imageIntersection, 
                                                    tf::Vector3& transformedIntersection,
                                                    tf::Vector3& cameraDirectionOffset) const
{
    double transformCoefficient = std::abs(transformedIntersection.z()) / cameraInfo_.focalLength;
    // Center intersections and swap axes.
    tf::Vector3 camera2intersection;
    camera2intersection.setX((imageIntersection.x() - 320.0) * transformCoefficient);
    camera2intersection.setY((imageIntersection.y() - 240.0) * transformCoefficient);
    camera2intersection.setZ(transformedIntersection.z());
    transformedIntersection = state_.fcu2camera * camera2intersection;
    //transformedIntersection += cameraDirectionOffset;
}



void IntersectionTransform::publishTransformedIntersections(const std::vector<Eigen::Vector2f>& imageIntersections,
                                                            const std::vector<tf::Vector3>& transformedIntersections,
                                                            const ros::Time stamp)
{
    if (imageIntersections.size() == transformedIntersections.size()) 
    {
        elikos_ros::IntersectionArray msg;
        // TODO: Use the stamp from the image.
        msg.header.stamp = stamp;
        msg.header.frame_id = "elikos_fcu";

        visualization_msgs::MarkerArray array;

        for (int i = 0; i < imageIntersections.size(); ++i) 
        {
            elikos_ros::Intersection intersection;
            intersection.imagePosition.x = imageIntersections[i].x();
            intersection.imagePosition.y = imageIntersections[i].y();

            intersection.arenaPosition.x = transformedIntersections[i].x();
            intersection.arenaPosition.y = transformedIntersections[i].y();
            intersection.arenaPosition.z = transformedIntersections[i].z();

            marker_.id = i;
            marker_.header.stamp = ros::Time::now();
            
            marker_.pose.position.x = transformedIntersections[i].x();
            marker_.pose.position.y = transformedIntersections[i].y();
            marker_.pose.position.z = transformedIntersections[i].z();

            array.markers.push_back(marker_);
            msg.intersections.push_back(intersection);
        }
        intersectionPub_.publish(msg);
        debugPub_.publish(array);
    }
}

}