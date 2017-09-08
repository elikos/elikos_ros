#ifndef PRE_PROCESSING_H
#define PRE_PROCESSING_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>


#include <ros/time.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Core>
#include <image_transport/image_transport.h>

#include "CameraInfo.h"
#include "QuadState.h"

namespace preprocessing
{

class PreProcessing
{
public:
    PreProcessing(const CameraInfo& cameraInfo, const QuadState& state);
    ~PreProcessing() = default;

    void preProcessImage(cv::Mat& raw, cv::Mat& preProcessed, cv::Mat& perspectiveTransform);
    void removePerspective(cv::Mat& input, cv::Mat& rectified, cv::Mat& perspectiveTransform);

private:

    Eigen::Matrix4f getPerspectiveProjectionTransform(double focalLength, double height, double length) const;


    const CameraInfo& cameraInfo_;
    const QuadState& state_;

    int undistortType_ = 1;

    cv::Mat distortionMap1_;
    cv::Mat distortionMap2_;
};

}

#endif // PRE_PROCESSING_H