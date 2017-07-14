#ifndef TRANSFORMATION_UTILS_H
#define TRANSFORMATION_UTILS_H

#include "geometry_msgs/PoseArray.h"
#include <opencv2/opencv.hpp>
#include "tf/tf.h"


namespace transformation_utils {

    geometry_msgs::PoseArray getFcu2TargetArray(const tf::StampedTransform& origin2fcu,
                                                const tf::StampedTransform& fcu2camera,
                                                const std::vector<cv::Point2f>& points,
                                                cv::Size dimensions,
                                                float hfov,
                                                float vfov);

    geometry_msgs::PoseStamped getFcu2Target(const tf::StampedTransform& origin2fcu,
                                             const tf::StampedTransform& fcu2camera,
                                             cv::Point2f point,
                                             cv::Size dimensions,
                                             float hfov,
                                             float vfov);

    geometry_msgs::Pose computeFcu2Target(const tf::StampedTransform& fcu2camera,
                                          const tf::Transform& origin2camera,
                                          cv::Point2f point,
                                          cv::Size dimensions,
                                          float hfov,
                                          float vfov);

    tf::Quaternion computeTurretRotation(cv::Point2f point, cv::Size imageDimension,
                                                             float hfov, float vfov);

    tf::Transform computeTurret2Target(tf::Transform origin2turret);

    geometry_msgs::PoseStamped getOrigin2Target(const tf::StampedTransform& origin2fcu,
                                         const tf::StampedTransform& fcu2camera,
                                         cv::Point2f point,
                                         cv::Size dimensions,
                                         float hfov,
                                         float vfov);

    geometry_msgs::Pose computeOrigin2Target(const tf::StampedTransform& fcu2camera,
                                      const tf::Transform& origin2camera,
                                      const tf::Transform& origin2fcu,
                                      cv::Point2f point,
                                      cv::Size dimensions,
                                      float hfov,
                                      float vfov);


};


#endif // TRANSFORMATION_UTILS_H
