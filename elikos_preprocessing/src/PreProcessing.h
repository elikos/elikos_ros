#ifndef PRE_PROCESSING_H
#define PRE_PROCESSING_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <ros/time.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>

namespace preprocessing
{

class PreProcessing
{
public:
    PreProcessing();
    ~PreProcessing() = default;

    void preProcessImage(const cv::Mat& raw, const ros::Time& stamp, cv::Mat& preProcessed, cv::Mat& inverseTransform);
    cv::Mat removePerspective(const cv::Mat& input, cv::Mat& rectified, const ros::Time& imageTime) const;

    Eigen::Vector2f translate(const Eigen::Vector2f& v, const Eigen::Vector2f& translation) const;
    Eigen::Vector2f rotate(const Eigen::Vector2f& v, double theta) const;

    void setRollPitch(double roll, double pitch);
    void setFocalLength(double focalLength);

private:

    Eigen::Matrix4f getPerspectiveProjectionTransform(double focalLength, double height, double length) const;

    tf::TransformListener tfListener_;
    ros::NodeHandle nh_;

    double focalLength_ = 0;
    double roll_ = 0.0;
    double pitch_ = 0.0;
};


}

#endif // PRE_PROCESSING_H