
#include "PreProcessing.h"

#include <Eigen/Geometry>

#include <iostream>

#include <cmath>

namespace localization 
{

PreProcessing::PreProcessing()
{
    cv::Mat distortedCamera = (cv::Mat_<float>(3,3) << 422.918640,    0.000000,    350.119451,
            0.000000,  423.121112,    236.380265,
            0.000000,    0.000000,      1.000000);

    cv::Mat cameraDistortion = (cv::Mat_<float>(1,5) << -0.321590, 0.089597, 0.001090, -0.000489, 0.000000);

    cv::Mat undistortedCamera = cv::getOptimalNewCameraMatrix(distortedCamera, cameraDistortion, cv::Size(640, 480), 0);

    cv::initUndistortRectifyMap(distortedCamera, cameraDistortion, cv::Mat(), undistortedCamera,
                                cv::Size(640, 480), CV_32FC1, distortionMap1_, distortionMap2_);

    cv::namedWindow("PreProcessed", 1);
    cv::createTrackbar("white threshold", "PreProcessed", &whiteThreshold_, 255);
    cv::createTrackbar("undistort type", "PreProcessed", &undistortType_, 1);
}

void PreProcessing::preProcessImage(const cv::Mat& raw, const ros::Time& stamp, cv::Mat& preProcessed)
{

    /*
    tf::StampedTransform tf;
    try {
        tfListener_.lookupTransform("elikos_local_origin", "elikos_fcu", ros::Time(0), tf);

        tf::Matrix3x3 m(tf.getRotation());
        double yaw;
        m.getRPY(roll_, pitch_, yaw);
        std::cout << pitch_ << " : " << roll_ << std::endl;

    } catch (tf::TransformException e) {
         ROS_ERROR("%s", e.what());
    }
    */

    //undistort
    // Blur
    cv::Mat typeConverted;
    if (raw.type() != CV_8UC1) {
        cv::cvtColor(raw, typeConverted, CV_BGR2GRAY);
    } else {
        raw.copyTo(typeConverted);
    }

    cv::Mat undistorted = typeConverted;
    if (!undistortType_) {
        cv::remap(typeConverted, undistorted, distortionMap1_, distortionMap2_, CV_INTER_LINEAR);
    }

    cv::Mat perspective;
    removePerspective(undistorted, perspective);
    cv::imshow("undistorted", undistorted);
    cv::imshow("perspective", perspective);

    cv::Mat blured;
    cv::GaussianBlur(perspective, blured, cv::Size(7,7), 8, 8);


    cv::Mat eroded;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    cv::erode(blured, eroded, element, cv::Point(0), 8);

    cv::Mat thresholded;
    cv::threshold(blured, thresholded, whiteThreshold_, 255, CV_THRESH_BINARY);

    preProcessed = thresholded;

    cv::imshow("PreProcessed", preProcessed);
}

void PreProcessing::removePerspective(const cv::Mat& input, cv::Mat& rectified) const
{
    double roll, pitch, yaw;
    Eigen::Vector3f direction;
    try {
        tf::StampedTransform tf;
        tfListener_.lookupTransform("elikos_local_origin", "elikos_fcu", ros::Time(0), tf);
        tf::Matrix3x3 m(tf.getRotation());
        m.getRPY(roll, pitch, yaw);
        tf::Vector3 v = m * tf::Vector3(0.0, 0.0, 1.0);
        direction.x() = v.x();
        direction.y() = v.y();
        direction.z() = v.z();

    } catch (tf::TransformException e) {
         ROS_ERROR("%s", e.what());
    }

    double height = input.size().height;
    double width = input.size().width;

    Eigen::Vector2f src[4] {{0.0, 0.0}, {0.0, height}, { width, height }, { width, 0.0}};
    Eigen::Vector2f dst[4] {{0.0, 0.0}, {0.0, height}, { width, height }, { width, 0.0}};
    
    double f = 423.0;
    double HFOV = std::atan( width / (2 * f));
    double VFOV = std::atan( height / (2 * f));
    
    Eigen::Vector2f leftRotationPoint = src[0] + (src[1] - src[0]) / 2.0;
    Eigen::Vector2f rightRotationPoint = src[3] + (src[2] - src[3]) / 2.0;;
    Eigen::Vector2f center = (src[2] - src[0]) / 2.0;

    float theta = std::atan2(direction.y(), -direction.x());
    if (std::isnan(theta)) {
        theta = 0.0;
    }

    float phi = std::atan2(std::sqrt(direction.x() * direction.x() + direction.y() * direction.y()), 1);

    Eigen::Rotation2D<float> R(phi);
    Eigen::Rotation2D<float> cr(theta);

    for (int i = 0; i < 4; ++i) 
    {
        dst[i] = cr * (dst[i] - center) + center;
    }
    
    leftRotationPoint = cr * (leftRotationPoint - center) + center;
    rightRotationPoint = cr * (rightRotationPoint - center) + center;

    src[0] = R.inverse() * (dst[0] - leftRotationPoint) + leftRotationPoint;
    src[1] = R.inverse() * (dst[1] - leftRotationPoint) + leftRotationPoint;
    src[2] = R * (dst[2] - rightRotationPoint) + rightRotationPoint;
    src[3] = R * (dst[3] - rightRotationPoint) + rightRotationPoint;

    cv::Point2f tSrc[4], tDst[4];
    for ( int i = 0; i < 4; ++i) 
    {
        tSrc[i] = { src[i].x(), src[i].y() };
        tDst[i] = { dst[i].x(), dst[i].y() };
    }

    cv::Mat perspectiveTransform = cv::getPerspectiveTransform(tSrc, tDst);

    cv::warpPerspective(input, rectified, perspectiveTransform, input.size());

    cv::circle(rectified, { (int)(src[0].x()), (int)(src[0].y()) }, 5, cv::Scalar(0, 200 ,0), -1);
    cv::circle(rectified, { (int)(src[1].x()), (int)(src[1].y()) }, 5, cv::Scalar(0, 200 ,0), -1);
    cv::circle(rectified, { (int)(src[2].x()), (int)(src[2].y()) }, 5, cv::Scalar(0, 200 ,0), -1);
    cv::circle(rectified, { (int)(src[3].x()), (int)(src[3].y()) }, 5, cv::Scalar(0, 200 ,0), -1);

    cv::circle(rectified, { (int)(dst[0].x()), (int)(dst[0].y()) }, 5, cv::Scalar(0, 100 ,0), -1);
    cv::circle(rectified, { (int)(dst[1].x()), (int)(dst[1].y()) }, 5, cv::Scalar(0, 100 ,0), -1);
    cv::circle(rectified, { (int)(dst[2].x()), (int)(dst[2].y()) }, 5, cv::Scalar(0, 100 ,0), -1);
    cv::circle(rectified, { (int)(dst[3].x()), (int)(dst[3].y()) }, 5, cv::Scalar(0, 100 ,0), -1);
}

void PreProcessing::showCalibTrackBars()
{

}

Eigen::Vector2f PreProcessing::translate(const Eigen::Vector2f& v, const Eigen::Vector2f& translation) const
{
    return v + translation;
}

Eigen::Vector2f PreProcessing::rotate(const Eigen::Vector2f& v, double theta) const
{
    double cosTheta = cosf(theta);
    double sinTheta = sinf(theta);

    Eigen::Vector2f rotated;
    rotated.x() = v.x() * cosTheta - v.y() * sinTheta;
    rotated.y() = v.x() * sinTheta + v.y() * cosTheta;

    return rotated;
}

}