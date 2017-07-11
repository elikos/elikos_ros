#include <iostream>
#include <tf/transform_broadcaster.h>

#include "tf/tf.h"


#include "QuadState.h"

#include "TransformationUtils.h"

#include "IntersectionTransform.h"

namespace localization {

IntersectionTransform::IntersectionTransform(const CameraInfo& cameraInfo, const QuadState& state)
    : cameraInfo_(cameraInfo), state_(state), imageIntersectionsPointCloud_(new pcl::PointCloud<pcl::PointXY>()), lastDetectionPointCloud_(new pcl::PointCloud<pcl::PointXY>())
{
    // TODO: Set epsilon for kdtree here maybe ? ...
    ros::NodeHandle nh;

    intersectionPub_ = nh.advertise<elikos_ros::IntersectionArray>(cameraInfo_.name + "/intersections", 1);
    debugPub_ = nh.advertise<visualization_msgs::MarkerArray>(cameraInfo_.name + "/intersection_debug", 1);

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
    marker_.color.r = 1.0;
    marker_.color.g = 0.0;
    marker_.color.b = 0.0;
}


void IntersectionTransform::updateKDTree(const std::vector<Eigen::Vector2f>& imageIntersections)
{
    imageIntersectionsPointCloud_->clear();
    for (int i = 0; i < imageIntersections.size(); ++i)
    {
        imageIntersectionsPointCloud_->push_back({ imageIntersections[i].x(), imageIntersections[i].y() });
    }
    imageIntersectionsTree_.setInputCloud(imageIntersectionsPointCloud_);
}

void IntersectionTransform::transformIntersections(const std::vector<Eigen::Vector2f>& imageIntersections,
                                                   const cv::Mat& perspectiveTransform,
                                                   cv::Size imageSize)
{
    if (imageIntersections.size() >= 1)
    {
	    updateKDTree(imageIntersections);
	    tf::Vector3 fcu2camera = tf::quatRotate(state_.getOrigin2Fcu().getRotation(), state_.getFcu2Camera().getOrigin());

        tf::Transform origin2camera = state_.getFcu2Camera();

        tf::Vector3 camDirection = tf::quatRotate(origin2camera.getRotation(), tf::Vector3(0.0, 0.0, 1.0));

        camDirection.normalize();

        float S = std::cos(std::atan(std::sqrt(std::pow(camDirection.x(), 2) + std::pow(camDirection.y(), 2)) / camDirection.z()));

	    double z = S * -estimateAltitude(imageIntersections) + fcu2camera.z();

	    std::vector<cv::Point2f> dst;
	    std::vector<cv::Point2f> src;
	    for (int i = 0; i < imageIntersections.size(); ++i)
	    {
		    src.push_back(cv::Point2f(imageIntersections[i].x(), imageIntersections[i].y()));
	    }
	    cv::perspectiveTransform(src, dst, perspectiveTransform);
	    geometry_msgs::PoseArray intersections = transformation_utils::getFcu2TargetArray(state_.getOrigin2Fcu(),
										     state_.getFcu2Camera(), dst, imageSize,
										     cameraInfo_.hfov, cameraInfo_.vfov);
	    for (int i = 0; i < intersections.poses.size(); ++i) {
		    intersections.poses[i].position.z = z;
	    }

        estimateQuadState(intersections);
	    publishTransformedIntersections(dst, intersections);
    } else {
        publishTransformedIntersections(std::vector<cv::Point2f>(), geometry_msgs::PoseArray());
    }
}

double IntersectionTransform::estimateAltitude(const std::vector<Eigen::Vector2f>& imageIntersections)
{
    double estimate = 0.0;
    double totalHeight = 0.0;
    int sampleSize = 0;

    const tf::Vector3& currentPosition = state_.getOrigin2Fcu().getOrigin();

    if (imageIntersections.size() > 1) 
    {
        std::vector<int> indices(2);
        std::vector<float> distances(2);
        for (int i = 0; i < imageIntersections.size(); ++i)
        {
            imageIntersectionsTree_.nearestKSearch(imageIntersectionsPointCloud_->at(i), 2, indices, distances);
            // Take the greatest since the smallest is the same point
            double imageDistance = (distances[0] > distances[1]) ? distances[0] : distances[1];

            // Compute local height estimate and error.
            double height = GRID_SIDE_LENGTH_M * (cameraInfo_.focalLength / std::sqrt(imageDistance));
            double error = std::abs(height - currentPosition.z());

            totalHeight += height;
            ++sampleSize;
            estimate = totalHeight / sampleSize;
        }
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
    camera2intersection.setY(-(imageIntersection.y() - 240.0) * transformCoefficient);
    camera2intersection.setZ(-transformedIntersection.z());

    transformedIntersection = camera2intersection;
}



void IntersectionTransform::publishTransformedIntersections(const std::vector<cv::Point2f>& imageIntersections,
                                                            const geometry_msgs::PoseArray& poseArray)
{
    if (imageIntersections.size() == poseArray.poses.size())
    {
        elikos_ros::IntersectionArray msg;
        msg.header.stamp = state_.getTimeStamp();
        msg.header.frame_id = "elikos_fcu";

        visualization_msgs::MarkerArray array;

        for (int i = 0; i < imageIntersections.size(); ++i) 
        {
            elikos_ros::Intersection intersection;
            intersection.imagePosition.x = imageIntersections[i].x;
            intersection.imagePosition.y = imageIntersections[i].y;

            intersection.arenaPosition = poseArray.poses[i].position;

            marker_.id = i;
            marker_.header.stamp = state_.getTimeStamp();
            marker_.pose = poseArray.poses[i];
            array.markers.push_back(marker_);
            msg.intersections.push_back(intersection);
        }
        intersectionPub_.publish(msg);
        debugPub_.publish(array);
    }
}

void IntersectionTransform::estimateQuadState(const geometry_msgs::PoseArray &intersections) {

    if (!lastDetection_.poses.empty())
    {
        tf::Vector3 translationEstimate = state_.getOrigin2Fcu().getOrigin() - lastState_.getOrigin();
        lastDetectionPointCloud_->clear();
        for (int i = 0; i < lastDetection_.poses.size(); ++i)
        {
            geometry_msgs::Point position = lastDetection_.poses[i].position;
            pcl::PointXY point;
            point.x = (float) (position.x + translationEstimate.x());
            point.y = (float) (position.y + translationEstimate.y());
            lastDetectionPointCloud_->push_back(point);
        }
        lastDetectionTree_.setInputCloud(lastDetectionPointCloud_);

        std::vector<bool> matched(intersections.poses.size(), false);
        std::vector<int> indices(1);
        std::vector<float> distances(1);

        tf::Vector3 averageTranslation;
        int nMatched = 0;

        double translationNormEstimate = translationEstimate.length();
        for (int i = 0; i < intersections.poses.size(); ++i)
        {
            pcl::PointXY point;
            geometry_msgs::Point position = intersections.poses[i].position;
            point.x = (float) position.x;
            point.y = (float) position.y;
            lastDetectionTree_.nearestKSearch(point, 1, indices, distances);
            double error = std::abs(translationNormEstimate - std::sqrt(distances[0]));
                std::string message = "Intersection [" + std::to_string(position.x) + ", " +
                        std::to_string(position.y) + "] match with [" +
                        std::to_string(lastDetection_.poses[indices[0]].position.x) + ", " +
                        std::to_string(lastDetection_.poses[indices[0]].position.y) + "] and error " +
                        std::to_string(error);
                ROS_WARN(message.c_str());
            if (!matched[i])
            {
                matched[i] = true;
                ++nMatched;
                averageTranslation.setX(averageTranslation.x() + (intersections.poses[i].position.x - lastDetection_.poses[i].position.x));
                averageTranslation.setY(averageTranslation.y() + (intersections.poses[i].position.y - lastDetection_.poses[i].position.y));
            }
        }
        averageTranslation.setX(averageTranslation.x() / (float) nMatched);
        averageTranslation.setY(averageTranslation.y() / (float) nMatched);
        averageTranslation.setZ(0.0);

        tf::StampedTransform transform(tf::Transform(tf::Quaternion::getIdentity(), state_.getOrigin2Fcu().getOrigin() + averageTranslation),
                                       state_.getTimeStamp(), "elikos_arena_origin", "elikos_vision_debug");
        tfPub_.sendTransform(transform);
    }

    lastDetection_ = intersections;
    lastState_ = state_.getOrigin2Fcu();
}

}
