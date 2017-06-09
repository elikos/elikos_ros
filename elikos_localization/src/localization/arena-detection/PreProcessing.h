#ifndef PRE_PROCESSING_H
#define PRE_PROCESSING_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <ros/time.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>

namespace localization
{

class PreProcessing
{
public:
    PreProcessing();
    ~PreProcessing() = default;

    void preProcessImage(const cv::Mat& raw, const ros::Time& stamp, cv::Mat& preProcessed);
    cv::Mat removePerspective(const cv::Mat& input, cv::Mat& rectified) const;
    cv::Mat removePerspectiveAndRotation(const cv::Mat& input, cv::Mat& rectified) const;
    void showCalibTrackBars();

    Eigen::Vector2f translate(const Eigen::Vector2f& v, const Eigen::Vector2f& translation) const;
    Eigen::Vector2f rotate(const Eigen::Vector2f& v, double theta) const;

    inline void setRollPitch(double roll, double pitch);

    inline void setFocalLength(double focalLength){focalLength_ = focalLength;}
private:

    Eigen::Matrix4f getPerspectiveProjectionTransform(double focalLength, double height, double length) const;

    tf::TransformListener tfListener_;

    double blurSigma = 0.0;
    int whiteThreshold_ = 149;
    int undistortType_ = 1;

    cv::Mat distortionMap1_;
    cv::Mat distortionMap2_;

    double roll_ = 0.0;
    double pitch_ = 0.0;

    double focalLength_ = 400;
};

inline void PreProcessing::setRollPitch(double roll, double pitch)
{
    roll_ = roll;
    pitch_ = pitch; 
}

}

#endif // PRE_PROCESSING_H