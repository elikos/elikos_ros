
#include "PreProcessing.h"

#include <Eigen/Geometry>
#include <Eigen/Core>

#include <iostream>

#include <cmath>
#include <cv_bridge/cv_bridge.h>

namespace preprocessing
{

PreProcessing::PreProcessing(const CameraInfo& cameraInfo, const QuadState& state)
    : cameraInfo_(cameraInfo), state_(state)
{
    cv::Mat distortedCamera = (cv::Mat_<float>(3,3) << 422.918640,    0.000000,    350.119451,
            0.000000,  423.121112,    236.380265,
            0.000000,    0.000000,      1.000000);

    cv::Mat cameraDistortion = (cv::Mat_<float>(1,5) << -0.321590, 0.089597, 0.001090, -0.000489, 0.000000);

    cv::Mat undistortedCamera = cv::getOptimalNewCameraMatrix(distortedCamera, cameraDistortion, cv::Size(640, 480), 0);

    cv::initUndistortRectifyMap(distortedCamera, cameraDistortion, cv::Mat(), undistortedCamera,
                                cv::Size(640, 480), CV_32FC1, distortionMap1_, distortionMap2_);
}

void PreProcessing::preProcessImage(cv::Mat& raw, cv::Mat& preProcessed, cv::Mat& perspectiveTransform)
{
    cv::Size size(640, 480);
    cv::Mat resized = raw;
    if (size != raw.size())
    {
        cv::resize(raw, resized, size);
    }

    cv::Mat undistorted = resized;
    if (!undistortType_) {
        cv::remap(resized, undistorted, distortionMap1_, distortionMap2_, CV_INTER_LINEAR);
    }

    cv::Mat perspective;
    removePerspective(undistorted, perspective, perspectiveTransform);

    cv::Mat blured;
    cv::GaussianBlur(perspective, blured, cv::Size(7,7), 8, 8);

    preProcessed = blured;
}

void PreProcessing::removePerspective(cv::Mat& input, cv::Mat& rectified, cv::Mat& toPerspective)
{
    double roll, pitch, yaw = 0.0;

    tf::Quaternion q = tf::Quaternion::getIdentity();
    q.setRPY(CV_PI, 0.0, 0.0);
    tf::Transform z2x(q);

    tf::Transform origin2fcu = tf::Transform(state_.getOrigin2Fcu().getRotation());
    tf::Transform fcu2camera = tf::Transform(state_.getFcu2Camera().getRotation());

    tf::Transform debug = origin2fcu * fcu2camera * z2x;

    tf::Matrix3x3 n(debug.getRotation());
    n.getRPY(roll, pitch, yaw);

    Eigen::Matrix3f r = (Eigen::AngleAxisf(roll,   Eigen::Vector3f::UnitX()) * 
                         Eigen::AngleAxisf(-pitch,  Eigen::Vector3f::UnitY()) *
                         Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ())).toRotationMatrix();

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
    double f = cameraInfo_.focalLength;
    double HFOV = cameraInfo_.hfov;
    double VFOV = cameraInfo_.vfov;

    Eigen::Vector4f src[4] { Eigen::Vector4f( 1.0,  1.0, 0.0, 1.0), 
                             Eigen::Vector4f(-1.0,  1.0, 0.0, 1.0), 
                             Eigen::Vector4f(-1.0, -1.0, 0.0, 1.0), 
                             Eigen::Vector4f( 1.0, -1.0, 0.0, 1.0) };

    Eigen::Vector4f dst[4] { Eigen::Vector4f( 1.0,  1.0, 0.0, 1.0), 
                             Eigen::Vector4f(-1.0,  1.0, 0.0, 1.0), 
                             Eigen::Vector4f(-1.0, -1.0, 0.0, 1.0), 
                             Eigen::Vector4f( 1.0, -1.0, 0.0, 1.0) };



    Eigen::Matrix4f P = getPerspectiveProjectionTransform(f, width, height); 
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T(2, 3) = -1.0;

    Eigen::Matrix4f perspectiveTransform = P * T * R;
    Eigen::Matrix4f orthoTransform =  P * T;

    for (int i = 0; i < 4; ++i)
    {
        dst[i] = orthoTransform * dst[i];
        dst[i] /= dst[i][3];

        src[i] = perspectiveTransform * src[i];
        src[i] /= src[i][3];
    }

    cv::Point2f tSrc[4], tDst[4];
    for (int i = 0; i < 4; ++i) 
    {
        tSrc[i] = cv::Point2f(src[i].x() * width / 2.0 + width / 2.0, src[i].y() * height / 2.0 + height / 2.0);
        tDst[i] = cv::Point2f(dst[i].x() * width / 2.0 + width / 2.0, dst[i].y() * height / 2.0 + height / 2.0);
    }

    cv::warpPerspective(input, rectified, cv::getPerspectiveTransform(tSrc, tDst), input.size());
    toPerspective = cv::getPerspectiveTransform(tDst, tSrc);
}

Eigen::Matrix4f PreProcessing::getPerspectiveProjectionTransform(double focalLength, double width, double height) const
{
    Eigen::Matrix4f m = Eigen::Matrix4f::Zero();
    m(0, 0) = 2 * focalLength / width;
    m(1, 1) = 2 * focalLength / height;
    m(3, 2) = -1;

    return m;
}

}