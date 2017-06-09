
#include "PreProcessing.h"

#include <Eigen/Geometry>
#include <Eigen/Core>

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
    removePerspectiveAndRotation(undistorted, perspective);
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

Eigen::Matrix4f createProjectionMatrix(double n) {
    Eigen::Matrix4f p = Eigen::Matrix4f::Zero();
    p(0, 0) = 1;
    p(1, 1) = 1;
    p(2, 2) = 1;
    p(3, 2) = 1/n;
    return p;
}

Eigen::Vector4f homogenize(const Eigen::Vector4f& vector){
    return vector / vector[3];
}

cv::Mat PreProcessing::removePerspectiveAndRotation(const cv::Mat& input, cv::Mat& rectified) const {
    Eigen::Matrix4f cameraRotation = Eigen::Matrix4f::Zero();

    Eigen::Vector4f direction;
    try {
        tf::StampedTransform tf;
        tfListener_.lookupTransform("elikos_local_origin", "elikos_ffmv_bottom", ros::Time(0), tf);

        tf::Matrix3x3 m(tf.getRotation());

        tf::Vector3 v = m * tf::Vector3(0.0, 0.0, 1.0);
        direction.x() = v.x();
        direction.y() = v.y();
        direction.z() = v.z();
        direction[3] = 1.0;

        cameraRotation(3, 3) = 1;
        for (int i = 0; i < 3; ++i) 
        {
            for (int j = 0; j < 3; j++) 
            {
                cameraRotation(i, j) = m[i][j];
            }
        }
    } catch (tf::TransformException e) {
         ROS_ERROR("%s", e.what());
    }

    double height = input.size().height;
    double width = input.size().width;

    double hheight = height/2.0;
    double hwidth = width/2.0;
    
    double f = focalLength_;

    Eigen::Vector4f sourcePoints[] = {
        Eigen::Vector4f( hwidth,  hheight, f, 1.0),
        Eigen::Vector4f(-hwidth,  hheight, f, 1.0),
        Eigen::Vector4f(-hwidth, -hheight, f, 1.0),
        Eigen::Vector4f( hwidth, -hheight, f, 1.0)
    };
    
    Eigen::Vector4f destinationPoints[4];

    Eigen::Matrix4f projectionMatrix = createProjectionMatrix(-1);
    Eigen::Vector4f center = homogenize(projectionMatrix * direction);

    Eigen::Vector2f scale = Eigen::Vector2f(1/0., 1/0.);

    //Tourner les points et les projeter sur le sol (on suppose que le sol est a 1 de distance)
    for(int i = 0; i < 4; ++i){
        destinationPoints[i] = homogenize(projectionMatrix * cameraRotation * sourcePoints[i]) - center;

        double scaleX = abs(hwidth / destinationPoints[i].x());
        if(scaleX < scale[0]){
            scale[0] = scaleX;
        }
        
        double scaleY = abs(hheight / destinationPoints[i].y());
        if(scaleY < scale[1]){
            scale[1] = scaleY;
        }
        /* CODE POUR SCALE UNIFORME (Pas pret)
        double angleFromCenter = atan2(destinationPoints[i][1], destinationPoints[i][0]);
        if (-angleUpRight <= angleFromCenter && angleFromCenter <= angleUpRight) {
            //DROIT
            projectedPoint[0] = hwidth;
            projectedPoint[1] = (hwidth / destinationPoints[i].x()) * destinationPoints[i].y();
        } else if (angleUpRight <= angleFromCenter && angleFromCenter <= PI - angleUpRight) {
            //HAUT
            projectedPoint[0] = (hheight / destinationPoints[i].y()) * destinationPoints[i].x();
            projectedPoint[1] = hheight;
        } else if (- PI + angleUpRight <= angleFromCenter && angleFromCenter <= - angleUpRight) {
            //BAS
            projectedPoint[0] = (-hheight / destinationPoints[i].y()) * destinationPoints[i].x();
            projectedPoint[1] = -hheight;
        } else {
            //GAUCHE
            projectedPoint[0] = -hwidth;
            projectedPoint[1] = (-hwidth / destinationPoints[i].x()) * destinationPoints[i].y();
        }
        double distance = projectedPoint.norm();
        
        if (distance > maxDistance) maxDistance = distance;
        */
    }

    for (int i = 0; i < 4; ++i){
        destinationPoints[i][0] *= scale[0];
        destinationPoints[i][1] *= scale[1];
    }

    cv::Point2f tSrc[4], tDst[4];
    for (int i = 0; i < 4; ++i) 
    {
        tSrc[i] = cv::Point2f(sourcePoints[i].x() + hwidth, sourcePoints[i].y() + hheight);
        tDst[i] = cv::Point2f(destinationPoints[i].x() + hwidth, destinationPoints[i].y() + hheight);
    }

    cv::Mat perspectiveTransform = cv::getPerspectiveTransform(tSrc, tDst);
    cv::warpPerspective(input, rectified, perspectiveTransform, input.size());

    return cv::getPerspectiveTransform(tDst, tSrc);
}

cv::Mat PreProcessing::removePerspective(const cv::Mat& input, cv::Mat& rectified) const
{
    double roll, pitch, yaw = 0.0;
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
    //pitch -= CV_PI / 2.0  - 0.4712;
    

    Eigen::Matrix3f r = (Eigen::AngleAxisf(-pitch, Eigen::Vector3f::UnitX()) * 
                         Eigen::AngleAxisf(-roll,  Eigen::Vector3f::UnitY())).toRotationMatrix();
                            
    Eigen::Matrix4f R = Eigen::Matrix4f::Zero();
    R(3, 3) = 1;
    for (int i = 0; i < 3; ++i) 
    {
        for (int j = 0; j < 3; j++) 
        {
            R(i, j) = r(i, j);
        }
    }

    double height = input.size().height;
    double width = input.size().width;
    double f = focalLength_;
    double HFOV = std::atan( width / (2 * f));
    double VFOV = std::atan( height / (2 * f));

    Eigen::Vector4f src[4] { Eigen::Vector4f( 1.0,  1.0, 0.0, 1.0), 
                             Eigen::Vector4f(-1.0,  1.0, 0.0, 1.0), 
                             Eigen::Vector4f(-1.0, -1.0, 0.0, 1.0), 
                             Eigen::Vector4f( 1.0, -1.0, 0.0, 1.0) };

    Eigen::Vector4f dst[4] { Eigen::Vector4f( 1.0,  1.0, 0.0, 1.0), 
                             Eigen::Vector4f(-1.0,  1.0, 0.0, 1.0), 
                             Eigen::Vector4f(-1.0, -1.0, 0.0, 1.0), 
                             Eigen::Vector4f( 1.0, -1.0, 0.0, 1.0) };
   
    float S = std::cos(pitch);

    Eigen::Matrix4f P = getPerspectiveProjectionTransform(f, width, height); 
    Eigen::Translation<float, 4> T(Eigen::Vector4f(0.0, 0.0, -1.0, 0.0));

    for (int i = 0; i < 4; ++i) 
    {
        dst[i] = T * dst[i];
        dst[i] = P * dst[i];
        dst[i] /= dst[i][3];

        //src[i] = S * src[i];
        //src[i].x() *= S;
        //src[i].y() *= S;
        src[i] = R * src[i];
        src[i] = T * src[i];
        //src[i].z() *= S;
        src[i] = P * src[i];
        //src[i] = P * T * R * dst[i];
        src[i] /= src[i][3];
    }

    cv::Point2f tSrc[4], tDst[4];
    for (int i = 0; i < 4; ++i) 
    {
        tSrc[i] = cv::Point2f(src[i].x() * width / 2.0 + width / 2.0, src[i].y() * height / 2.0 + height / 2.0);
        tDst[i] = cv::Point2f(dst[i].x() * width / 2.0 + width / 2.0, dst[i].y() * height / 2.0 + height / 2.0);
    }

    cv::Mat perspectiveTransform = cv::getPerspectiveTransform(tSrc, tDst);
    cv::warpPerspective(input, rectified, perspectiveTransform, input.size());

    //rectified = input.clone();
    for (int i = 0; i < 4; ++i) 
    {
        cv::circle(rectified, tSrc[i], 5, cv::Scalar(0, 200 ,0), -1);
        cv::circle(rectified, tDst[i], 5, cv::Scalar(0, 100 ,0), -1);
    }

    return cv::getPerspectiveTransform(tDst, tSrc);
}

Eigen::Matrix4f PreProcessing::getPerspectiveProjectionTransform(double focalLength, double width, double height) const
{
    Eigen::Matrix4f m = Eigen::Matrix4f::Zero();
    m(0, 0) = 2 * focalLength / width;
    m(1, 1) = 2 * focalLength / height;
    m(3, 2) = -1;

    return m;
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