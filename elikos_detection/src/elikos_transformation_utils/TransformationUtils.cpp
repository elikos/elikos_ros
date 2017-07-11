//
// Created by elikos on 23/06/17.
//

#include "tf/transform_datatypes.h"

#include "TransformationUtils.h"

#include <cmath>
#include <tf/transform_broadcaster.h>

namespace transformation_utils {

geometry_msgs::PoseArray getFcu2TargetArray(const tf::StampedTransform& origin2fcu,
                                                const tf::StampedTransform& fcu2camera,
                                                const std::vector<cv::Point2f>& points,
                                                cv::Size dimensions,
                                                float hfov,
                                                float vfov)
{
    tf::Transform origin2camera = origin2fcu * fcu2camera;
    geometry_msgs::PoseArray poseArray;
    poseArray.header.frame_id = origin2fcu.child_frame_id_;
    poseArray.header.stamp = origin2fcu.stamp_;
    for (int i = 0; i  < points.size(); ++i)
    {
        poseArray.poses.push_back(computeFcu2Target(fcu2camera, origin2camera, points[i], dimensions, hfov, vfov));
    }
    return poseArray;
}

geometry_msgs::PoseStamped getFcu2Target(const tf::StampedTransform& origin2fcu,
                                         const tf::StampedTransform& fcu2camera,
                                         cv::Point2f point,
                                         cv::Size dimensions,
                                         float hfov,
                                         float vfov)
{
    tf::Transform origin2camera = origin2fcu * fcu2camera;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = origin2fcu.child_frame_id_;
    pose.header.stamp = origin2fcu.stamp_;
    pose.pose = computeFcu2Target(fcu2camera, origin2camera, point, dimensions, hfov, vfov);
    return pose;
}

geometry_msgs::PoseStamped getOrigin2Target(const tf::StampedTransform& origin2fcu,
                                         const tf::StampedTransform& fcu2camera,
                                         cv::Point2f point,
                                         cv::Size dimensions,
                                         float hfov,
                                         float vfov)
{
    tf::Transform origin2camera = origin2fcu * fcu2camera;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = origin2fcu.frame_id_;
    pose.header.stamp = origin2fcu.stamp_;
    pose.pose = computeOrigin2Target(fcu2camera, origin2camera, origin2fcu, point, dimensions, hfov, vfov);
    return pose;
}

geometry_msgs::Pose computeFcu2Target(const tf::StampedTransform& fcu2camera,
                                      const tf::Transform& origin2camera,
                                      cv::Point2f point,
                                      cv::Size dimensions,
                                      float hfov,
                                      float vfov)
{
    //Define the camera2turret turret transform
    tf::Transform camera2turret = tf::Transform::getIdentity();

    //Same origin than the camera
    camera2turret.setOrigin(tf::Vector3(0, 0, 0));

    //Rotation in function of the detection with computer vision
    tf::Quaternion rotation = computeTurretRotation(point, dimensions, hfov, vfov);
    camera2turret.setRotation(rotation);

    // Compute the origin to turret transform
    tf::Transform origin2turret = origin2camera * camera2turret;

    // Find the robot2turret transform
    tf::Transform turret2target = computeTurret2Target(origin2turret);

    //Compute the robot poses
    tf::Transform fcu2target = fcu2camera * camera2turret * turret2target ;

    geometry_msgs::Pose pose;
    tf::quaternionTFToMsg(fcu2target.getRotation(), pose.orientation);
    tf::pointTFToMsg(fcu2target.getOrigin(), pose.position);
    return pose;
}

geometry_msgs::Pose computeOrigin2Target(const tf::StampedTransform& fcu2camera,
                                      const tf::Transform& origin2camera,
                                      const tf::Transform& origin2fcu,
                                      cv::Point2f point,
                                      cv::Size dimensions,
                                      float hfov,
                                      float vfov)
{
    //Define the camera2turret turret transform
    tf::Transform camera2turret = tf::Transform::getIdentity();

    //Same origin than the camera
    camera2turret.setOrigin(tf::Vector3(0, 0, 0));

    //Rotation in function of the detection with computer vision
    tf::Quaternion rotation = computeTurretRotation(point, dimensions, hfov, vfov);
    camera2turret.setRotation(rotation);

    // Compute the origin to turret transform
    tf::Transform origin2turret = origin2camera * camera2turret;

    // Find the robot2turret transform
    tf::Transform turret2target = computeTurret2Target(origin2turret);

    //Compute the robot poses
    tf::Transform origin2target = origin2fcu * fcu2camera * camera2turret * turret2target ;

    geometry_msgs::Pose pose;
    tf::quaternionTFToMsg(origin2target.getRotation(), pose.orientation);
    tf::pointTFToMsg(origin2target.getOrigin(), pose.position);
    return pose;
}

tf::Transform computeTurret2Target(tf::Transform origin2turret)
{
    // Get the smallest angle between the turret and the z axis
    // - First get the vector pointing towards the z axis of the turret
    tf::Vector3 vect_z = tf::quatRotate(origin2turret.getRotation(), tf::Vector3(0, 0, 1));
    // - Then find it's angle with the camera's resting position (-z)
    tf::Vector3 zAxis(0, 0, -1);
    double zAxis_turret_angle = zAxis.angle(vect_z);

    // Initiallization on the robot frame coordinate
    tf::Transform robotFrame = tf::Transform::getIdentity();
    robotFrame.setOrigin(tf::Vector3(0, 0, 0));

    tf::Transform lol2 = tf::Transform::getIdentity();
    lol2.setOrigin(tf::Vector3(0, 0, 0));
    lol2.setRotation(origin2turret.getRotation());
    // Get distance from turret to target (using angle and altitude) Should we use a mavros topic? nope
    double altitude = origin2turret.getOrigin().getZ();
    tf::Vector3 direction = lol2.inverse() * tf::Vector3(0,0,1);
    double distance_from_robot = altitude*(pow(direction.x(), 2) +pow(direction.y(), 2) +pow(direction.z(), 2))/-direction.z();

            //altitude / cos(zAxis_turret_angle);
    robotFrame.setOrigin(tf::Vector3(0, 0, distance_from_robot));
    //Set the orientation
    //the robots are on the ground, so we use the local_origin frame
    tf::Quaternion orientation = origin2turret.inverse().getRotation();
    robotFrame.setRotation(orientation);

    return robotFrame;
}

tf::Quaternion computeTurretRotation(cv::Point2f point, cv::Size imageSize,
                                                         float hfov, float vfov)
{
    float f = imageSize.width / (2.f * tan(hfov / 2));
    //initialization
    //tf::Quaternion rotation = tf::createIdentityQuaternion();
    //compute angles
    //double roll = -((double) (point.y - imageSize.height / 2) / (double) imageSize.height) * vfov;rotation
    //double pitch = ((double) (point.x - imageSize.width / 2) / (double) imageSize.width) * hfov;
    //double roll = -(double) atan2((point.y - imageSize.height / 2), f);
    //double pitch = (double) atan2((point.x - imageSize.width / 2), f);
    //set rotation
    //rotation.setRPY(roll , pitch, (double) 0.0);
    tf::Vector3 camera(0, 0, f);
    tf::Vector3 turret(point.x-imageSize.width / 2, point.y - imageSize.height / 2, f);
    float k_cos_theta = camera.dot(turret);
    float k = std::sqrt(camera.length2() * turret.length2());

    tf::Vector3 result;

    //if((int)(k_cos_theta/k) == -1)
    //{
    //    result = result.normalize();
    //    rotation
    //}

    result = camera.cross(turret);

    tf::Quaternion rotation(result.x(), result.y(), result.z(), k_cos_theta+k);

    return rotation.normalize();
}

}
